import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
df = pd.read_csv("txop_2.csv")
plt.rcParams.update({
    # "text.usetex": True,
    # "font.family": "serif",
    # "font.serif": ["Times New Roman"],
    'figure.dpi': 300,
    'savefig.dpi': 300,
    "font.size": 16
    })
print(df)
df["TxopBegin"] = df["TxopBegin"] / 1e6
df["TxopEnd"] = df["TxopEnd"] / 1e6

txop = {0:[], 1:[]}
txopy = {0:[], 1:[]}
for i in range(2):
    df2 = df[df["LinkId"] == i]
    mx = df2["AmpduLength"].max()
    prev = 0
    for j in range(len(df2)):
        a = df2.iloc[j][1]
        b = df2.iloc[j][2]
        if a < 4.5 or a > 4.7: 
            continue
        if prev > 0:
            txop[i] += list(np.linspace(prev, a, 100))
            txopy[i] += [0] * 100
        prev = b
        val = 0.5 if i == 0 else 1
        txop[i] += list(np.linspace(a, b, 100))
        txopy[i] += [val] * 100
# plt.subplot()
plt.figure(1, figsize=(50, 20))
plt.plot(txop[0], txopy[0], label = "2.4 G")
plt.plot(txop[1], txopy[1], linestyle = "--", label = "5 G")
plt.legend()
plt.xlabel("Time [ms]", fontsize = 16)
plt.ylabel("Txop", fontsize = 16)
# plt.ylim(0, 256)
plt.savefig("1.png")

