# Update Q-Table
        maxQ = np.max(q_table[tuple(next_state)])
        q_table[tuple(state)][action] += alpha * (reward +
                                                  gamma * maxQ - q_table[tuple(state)][action])