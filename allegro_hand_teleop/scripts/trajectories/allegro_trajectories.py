def main():
    filename = "allegro_trajectories3"

    with open(filename, 'r') as f:
        lines = f.readlines()

    reward_indices = [i for i, line in enumerate(lines) if "reward" in line]
    
    if reward_indices[-1] != len(lines)-1:
        reward_indices.append(len(lines)-1)

    for i in range(len(reward_indices)):
        start = 0 if i == 0 else reward_indices[i-1] + 1
        end = reward_indices[i]
        data = "".join(lines[start:end])
        data = data.replace("[", "")
        data = data.replace("]", "")
        with open(filename + "_" + str(i+1).zfill(2), "w") as f:
            f.write(data)


if __name__ == '__main__':
    main()

