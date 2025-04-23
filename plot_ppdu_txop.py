import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
df = pd.read_csv('PPDU.csv')
# df['type'] = df['type'].astype(str)

color_map = {
    '0': '#1f77b4',      # 蓝色
    '1': '#2ca02c',      # 绿色
    'OBSS': '#ff7f0e',   # 橙色
}

def get_y(type_):
    if type_ == '1':
        return 1
    else:
        return 0

fig, ax = plt.subplots(figsize=(16, 4))

begin = 1080000
end = 1090000
for idx, row in df.iterrows():
    x_start = row.iloc[1]
    x_end = row.iloc[2]
    if x_start > begin and x_start < end:
        width = x_end - x_start
        y = int(row.iloc[0])
        color = color_map.get(str(row.iloc[0]), 'gray')
        rect = plt.Rectangle((x_start, y-0.4), width, 0.8, color=color, alpha=0.7, edgecolor='black')
        ax.add_patch(rect)
        ax.text(x_start + width/2, y, str(row.iloc[3]),
                ha='center', va='center', fontsize=8, color='black')

# 读取txopPPDU.csv并画点
txop_df = pd.read_csv('./Txop.csv')
link_color = {0: 'purple', 1: 'red'}

for idx, row in txop_df.iterrows():
    link = row.iloc[0]
    txopstart = row.iloc[1]
    txopend = row.iloc[2]
    if txopstart > begin and txopstart < end:
        color = link_color.get(link, 'black')
        y_pos = 0.6 if link == 1 else -0.4
        # 三角形标txopstart
        ax.scatter(txopstart, y_pos, marker='^' , alpha=0.7, color=color, s=10, label=f'link{link} start' if idx == 0 else "")
        # 方形标txopend
        ax.scatter(txopend, y_pos, marker='.', color=color,alpha=0.7, s=10, label=f'link{link} end' if idx == 0 else "")

# 设置y轴
ax.set_yticks([0, 1])
ax.set_yticklabels(['2.4 G & OBSS', '5 G'])
ax.set_xlabel('Time')
ax.set_ylabel('Type')
ax.set_title('PPDU Timeline')

from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor=color_map['0'], edgecolor='black', label='2.4 G'),
    Patch(facecolor=color_map['1'], edgecolor='black', label='5 G'),
    Patch(facecolor=color_map['OBSS'], edgecolor='black', label='OBSS'),
    Patch(facecolor='purple', edgecolor='black', label='link0 txop'),
    Patch(facecolor='red', edgecolor='black', label='link1 txop'),
]
ax.legend(handles=legend_elements, loc='upper right')

ax.set_ylim(-0.7, 1.7)
ax.autoscale(enable=True, axis='x', tight=True)
plt.tight_layout()
plt.savefig('PPDU_Timeline.png', dpi=300)