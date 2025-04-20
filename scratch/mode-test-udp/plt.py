import pandas as pd

df1 = pd.read_csv("./Scene_2001_thpt.csv")
df2 = pd.read_csv("./Scene_2002_thpt.csv")
data1 = pd.read_csv("./Scene_2001_thpt.csv")[" Throughput(Mbps)"]
data2 = pd.read_csv("./Scene_2002_thpt.csv")[" Throughput(Mbps)"]
n = 0
for d in data1:
    if d > 0:
        n += 1
import matplotlib.pyplot as plt
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times New Roman"],
    'figure.dpi': 200,
    'savefig.dpi': 200,
    "font.size": 14
})
plt.figure(1)
plt.plot(range(n), data1[:n], label="seed = 2001")
plt.plot(range(n), data2[:n], label="seed = 2002")
plt.legend()
plt.xlabel("period")
plt.ylabel("Throughput (Mbps)")
plt.savefig("thpt.png")

rates = []
for i in range(n):
    r = abs(data1[i] - data2[i]) / max(data1[i], data2[i])
    rates.append(r)
plt.figure(2)
plt.plot(range(n), rates, label="delta")
plt.legend()
plt.xlabel("period")
plt.ylabel("delta")
plt.savefig("rate.png")

p1 = df2[" p1"]
p2 = df2[" p2"]
occ1 = df2[" Occupancy Rate 1"]
occ2 = df2[" Occupancy Rate 2"]
ans = []
for i in range(n):
    x = float(p1[i]) * float(occ1[i]) + float(p2[i]) * float(occ2[i])
    ans.append(x)
plt.figure(3)
plt.plot(range(len(ans)), ans, label="f", linestyle="--")
plt.plot(range(n), data1[:n]/100, label="seed = 2002")
plt.legend()
plt.xlabel("period")
# plt.ylabel("")
plt.savefig("ans2.png")
import numpy as np
g = []
txoptime1 = df2[" TxopTime1(us)"]
txoptime2 = df2[" TxopTime2(us)"]
txopcnt1 = df2[" TxopCnt1"]
txopcnt2 = df2[" TxopCnt2"]
for i in range(n):
    # x = abs(float(txoptime1[i]) * float(txopcnt1[i]) - (float(txoptime2[i]) * float(txopcnt2[i]))) / max(float(txoptime1[i]) * float(txopcnt1[i]), float(txoptime2[i]) * float(txopcnt2[i]))
    y = float(txoptime1[i]) * float(txopcnt1[i]) / (float(txoptime2[i]) * float(txopcnt2[i]))
    g.append(1 / (y-1)**2)
    
plt.figure(4)
plt.plot(range(n), g, label="g")
# plt.plot(range(n), data1[:n]/100, label="seed = 2001")
plt.xlabel("period")
plt.ylabel("g")
plt.savefig("g.png")