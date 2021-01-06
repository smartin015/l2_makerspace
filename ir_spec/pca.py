import numpy as np

with open("testdata.csv", "r") as f:
    data = f.read()

lines = data.split('\n')
nums = np.ndarray((len(lines), len(lines[0].split(','))))

names = set([l.split(',')[0].strip() for l in lines])
idxs = dict([(v,i) for (i,v) in enumerate(names)])

for (i,l) in enumerate(lines):
    if l.strip() == "":
        continue
    sp = l.split(',')
    print(sp)
    nums[i,0] = idxs[sp[0].strip()]
    # TODO i,1
    nums[i,2:] = [int(s) for s in sp[2:]]

print(nums)

print("PCA...")
import matplotlib.pyplot as plt
# from sklearn.decomposition import PCA
# pca = PCA(n_components=3)
from sklearn.decomposition import FastICA
pca = FastICA(n_components=3)

pca.fit(nums[:,2:])
# print(pca.explained_variance_ratio_)
# print(pca.singular_values_)

import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

import matplotlib.cm as cm
colors = cm.rainbow(np.linspace(0, 1, len(names)))
for i, xyz in enumerate(pca.transform(nums[:,2:])):
    ax.scatter(xyz[0], xyz[1], xyz[2],  color=colors[int(nums[i][0])])

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
