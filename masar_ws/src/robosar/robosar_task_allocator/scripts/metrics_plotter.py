import matplotlib.pyplot as plt

high_sot = [2424, 2464, 2897, 2344, 2376, 2034, 2458, 2632, 1806, 2458, 2836, 2606, 2213, 2922]
naive_sot = [3085, 2490, 2722, 3117, 3154, 3565, 3416, 3170, 3708, 4210, 3690, 3951, 3518]
sot = [high_sot, naive_sot]

plt.figure()
plt.boxplot(sot)
plt.xticks([1, 2], ['HIGH', 'Naive'])
plt.title("Sum of Times")
plt.xlabel("Task Allocator")
plt.ylabel("Time [s]")

plt.figure()
plt.hist(high_sot, 5)
plt.hist(naive_sot, 5)
plt.legend(['HIGH', 'Naive'])
plt.title("Sum of Times")
plt.xlabel("Task Allocator")
plt.ylabel("Time [s]")

plt.show()