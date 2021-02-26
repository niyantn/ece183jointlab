import matplotlib.pyplot as plt

fig = plt.figure()
plt.plot(range(10))
fig.savefig('temp.png', dpi=fig.dpi)