import numpy as np
import math

def q2rpy_rad(x,y,z,w):
    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*z))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    return r, p, y

def rpy2q_rad(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return qx, qy, qz, qw

def norm(x): return np.sqrt(np.sum([m*m for m in x]))

def body_to_earth_frame(ii, jj, kk):
    Cii, Cjj, Ckk=math.cos(ii), math.cos(jj), math.cos(kk)
    Sii, Sjj, Skk=math.sin(ii), math.sin(jj), math.sin(kk)

    R = np.array([[Ckk * Cjj, Ckk * Sjj * Sii - Skk * Cii, Ckk * Sjj * Cii + Skk * Sii],
                [Skk * Cjj, Skk * Sjj * Sii + Ckk * Cii, Skk * Sjj * Cii - Ckk * Sii],
                [-Sjj, Cjj * Sii, Cjj * Cii]])
    return R

def var_B2E(ii, jj, kk, xb, yb, zb):
    Cii, Cjj, Ckk=math.cos(ii), math.cos(jj), math.cos(kk)
    Sii, Sjj, Skk=math.sin(ii), math.sin(jj), math.sin(kk)

    R = np.array([[((Ckk*Sjj*Cii+Skk*Sii)*yb + (-Ckk*Sjj*Sii+Skk*Cii)*zb), (-Ckk*Sjj*xb + Ckk*Cjj*Sii*yb+Ckk*Cjj*Cii*zb), (-Skk*Cjj*xb+(-Skk*Sjj*Sii-Ckk*Cii)*yb+(-Skk*Sjj*Cii+Ckk*Sii)*zb)],
                [((Skk * Sjj * Cii - Ckk * Sii)*yb + (-Skk * Sjj * Sii - Ckk * Cii)*zb),(-Skk*Sjj*xb+Skk*Cjj*Sii*yb+Skk*Cjj*Cii*zb),(Ckk*Cjj*xb+(Ckk * Sjj * Sii - Skk * Cii)*yb + (Ckk * Sjj * Cii + Skk * Sii)*zb)],
                [Cjj*Cii*yb-Cjj*Sii*zb, -Cjj*xb-Sjj*Sii*yb-Sjj*Cii*zb, 0]])
    # print(R)
    return np.square(R)

def earth_to_body_frame(ii, jj, kk):
    return np.transpose(body_to_earth_frame(ii, jj, kk))

to_rad=lambda x: x*np.pi/180