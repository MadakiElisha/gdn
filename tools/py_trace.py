import numpy as np, pandas as pd, math
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import sys; sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain_map import TerrainMap

def skew(v): return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
def clamp(v,l):
    n=np.linalg.norm(v); return v*(l/n) if n>l else v

D=Path.home()/'gdn_workspace/data/flight001_csv'
LAT_C,LON_C=47.3977,8.5456; M_LAT=111320.0; M_LON=111320.0*np.cos(np.radians(LAT_C))
imu=pd.read_csv(D/'imu.csv'); lpos=pd.read_csv(D/'lpos.csv')
t0=imu['t'].iloc[0]; T=(imu['t'].to_numpy()-t0)/1e6
TL=(lpos['t'].to_numpy()-t0)/1e6
REF=np.column_stack([lpos['x'],lpos['y'],lpos['z']]).astype(float)
G=imu[['gx','gy','gz']].to_numpy(); A=imu[['ax','ay','az']].to_numpy()
spd=np.hypot(lpos['vx'],lpos['vy']).to_numpy()

# Pass 1: find alignment index
sa=sw=sa2=np.zeros(3); n=0; align_idx=-1
for i in range(len(T)):
    if np.linalg.norm(G[i])<0.05: sa+=A[i]; sw+=G[i]; sa2+=A[i]**2; n+=1
    else: sa=sw=sa2=np.zeros(3); n=0
    if n>=500:
        ma, mw = sa/n, sw/n; var=sa2/n-ma**2
        if np.linalg.norm(mw)<=0.02 and abs(np.linalg.norm(ma)-9.80665)<=0.5 and var.max()<=1.0:
            align_idx=i; break
        sa=sw=sa2=np.zeros(3); n=0

if align_idx<0:
    print("alignment failed"); sys.exit(1)

bg=mw; roll=np.arctan2(-ma[1],-ma[2]); pitch=np.arctan2(ma[0],np.hypot(ma[1],ma[2]))
q=R.from_euler('ZYX',[0,pitch,roll]); g=np.array([0,0,9.80665])
ba=ma-q.inv().apply(-g)
print(f"aligned at i={align_idx}, ba={np.linalg.norm(ba):.4f}, bg={np.linalg.norm(bg):.4f} deg/s")

# Pass 2: propagate from align_idx
pos=np.zeros(3); vel=np.zeros(3); x=np.zeros(15)
P=np.zeros((15,15)); P[0:3,0:3]=np.eye(3)*100; P[3:6,3:6]=np.eye(3); P[6:9,6:9]=np.eye(3)*(2*np.pi/180)**2
P[9:12,9:12]=np.eye(3)*0.01; P[12:15,12:15]=np.eye(3)*(0.1*np.pi/180)**2
Qc=np.zeros((15,15)); Qc[3:6,3:6]=np.eye(3)*0.02**2; Qc[6:9,6:9]=np.eye(3)*(0.02*np.pi/180)**2
Qc[9:12,9:12]=np.eye(3)*5e-4**2; Qc[12:15,12:15]=np.eye(3)*(0.01*np.pi/180)**2
Rtrn=144.0
tm=TerrainMap(); rng=np.random.default_rng(7)
fire_k=-1; meas=None; yaw_on=False; k=0

for i in range(align_idx+1, len(T)):
    dt=T[i]-T[i-1]
    if dt<=0: continue
    a_b=A[i]-ba; w_b=G[i]-bg
    Rm=q.as_matrix(); F=np.zeros((15,15))
    F[0:3,3:6]=np.eye(3); F[3:6,6:9]=-Rm@skew(a_b); F[3:6,9:12]=-Rm
    F[6:9,6:9]=-skew(w_b); F[6:9,12:15]=-np.eye(3)
    Phi=np.eye(15)+F*dt; P=Phi@P@Phi.T+Qc*dt
    q=q*R.from_rotvec(w_b*dt); vel=vel+(q.apply(a_b)+g)*dt; pos=pos+vel*dt
    
    # Fire measurement at 0.5 s grid
    kk=int(math.floor(T[i]*2))
    if kk!=fire_k:
        fire_k=kk
        lt=TL[np.abs(TL-T[i]).argmin()]; lr=REF[np.abs(TL-T[i]).argmin()]
        h_agl=max(-lr[2],0.0)
        n5=rng.normal(0,5); n2=rng.normal(0,2)
        meas=(tm.get_altitude(LAT_C+lr[0]/M_LAT, LON_C+lr[1]/M_LON)+h_agl+n5)-(h_agl+n2)
        
        # Transfer yaw
        j=np.abs(TL-T[i]).argmin()
        if not yaw_on and spd[j]>3.0 and abs(w_b[2])<0.05:
            yaw=math.atan2(lpos['vy'].iloc[j], lpos['vx'].iloc[j])
            rz=R.from_euler('z',yaw).as_matrix()
            q=R.from_euler('z',yaw)*q; pos=rz@pos; vel=rz@vel
            B=np.eye(15); B[0:3,0:3]=rz; B[3:6,3:6]=rz; P=B@P@B.T
            P[6:9,6:9]+=np.eye(3)*(np.pi/180)**2
            yaw_on=True
        
        # TRN update
        if yaw_on and meas is not None:
            lat_n=LAT_C+pos[0]/M_LAT; lon_n=LON_C+pos[1]/M_LON
            h0=tm.get_altitude(lat_n,lon_n)
            d=20.0
            gn=(tm.get_altitude(lat_n+d/M_LAT,lon_n)-tm.get_altitude(lat_n-d/M_LAT,lon_n))/(2*d)
            ge=(tm.get_altitude(lat_n,lon_n+d/M_LON)-tm.get_altitude(lat_n,lon_n-d/M_LON))/(2*d)
            if math.hypot(gn,ge)>=0.02:
                H=np.zeros((1,15)); H[0,0]=gn; H[0,1]=ge
                innov=meas-h0-H@x; S=(H@P@H.T)[0,0]+Rtrn
                pb=pos.copy()
                if S>0 and abs(innov)/math.sqrt(S)<=3.0 and abs(innov)<=40.0:
                    K=(P@H.T)/S; dx=(K*innov).ravel()
                    dx[6:9]=clamp(dx[6:9],0.2*np.pi/180); dx[9:12]=clamp(dx[9:12],0.005); dx[12:15]=clamp(dx[12:15],0.05*np.pi/180)
                    x+=dx; IKH=np.eye(15)-K@H; P=IKH@P@IKH.T+Rtrn*K@K.T
                    pos=pos+x[0:3]; vel=vel+x[3:6]; q=q*R.from_rotvec(x[6:9])
                    ba=ba+x[9:12]; bg=bg+x[12:15]
                    J=np.eye(15); J[6:9,6:9]=np.eye(3)-skew(x[6:9]); P=J@P@J.T; x=np.zeros(15)
                    if k<40: print(f"UPD k={k} meas={meas:.2f} h0={h0:.2f} innov={innov:.2f} S={S:.1f} acc=1 pb=({pb[0]:.1f},{pb[1]:.1f})")
                    k+=1
                else:
                    if k<40: print(f"UPD k={k} meas={meas:.2f} h0={h0:.2f} innov={innov:.2f} S={S:.1f} acc=0 pb=({pb[0]:.1f},{pb[1]:.1f})")
                    k+=1
