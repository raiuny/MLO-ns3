import pandas as pd
import matplotlib.pyplot as plt

# file = "./scratch/mode-test-udp-no-period/result_bw_20_160_mcs_6_11_interference_0.500000_0.000000_txoplimits_0_0_nss_2_thpt.csv"
file = "./scratch/mode-test-udp-no-period/result_bw_20_160_mcs_6_11_interference_0.000000_0.500000_txoplimits_0_0_nss_2_thpt.csv"
df = pd.read_csv(file)

# 创建图表和两个Y轴
fig, ax1 = plt.subplots(figsize=(12, 6))
ax2 = ax1.twinx()

# 绘制吞吐量相关曲线 (左Y轴)
ln1 = ax1.plot(df[' Time'], df[' Throughput(Mbps)'], 'o-', label='Total Throughput', color='blue')
ln2 = ax1.plot(df[' Time'], df[' throughput1'], 'o-', label='Throughput1', color='green')
ln3 = ax1.plot(df[' Time'], df[' throughput2'], 'o-', label='Throughput2', color='red')
ln4 = ax1.plot(df[' Time'], df[' datarate1'], '--', label='Datarate1', color='cyan')
ln5 = ax1.plot(df[' Time'], df[' datarate2'], '--', label='Datarate2', color='magenta')

# 绘制rate相关曲线 (右Y轴)
ln6 = ax2.plot(df[' Time'], df[' blocktimerate1'], 's:', label='Blocktimerate1', color='brown')
ln7 = ax2.plot(df[' Time'], df[' blocktimerate2'], 's:', label='Blocktimerate2', color='orange')
ln8 = ax2.plot(df[' Time'], df[' blockrate1'], 's:', label='Blockrate1', color='purple')
ln9 = ax2.plot(df[' Time'], df[' blockrate2'], 's:', label='Blockrate2', color='gray')

# 设置轴标签和范围
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Throughput/Datarate (Mbps)')
ax2.set_ylabel('Rate')
ax2.set_ylim(0, 1)

# 合并两个轴的图例
lns = ln1 + ln2 + ln3 + ln4 + ln5 + ln6 + ln7 + ln8 + ln9
labs = [l.get_label() for l in lns]
ax1.legend(lns, labs, loc='center right', bbox_to_anchor=(1.4, 0.5))

# 设置网格线
ax1.grid(True, alpha=0.3)

plt.title('Throughput and Rate Metrics')
plt.tight_layout()
plt.show()