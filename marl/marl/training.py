import torch
import numpy as np
import torch.nn as nn
import torch.optim as optim
from environment import MultiAgentEnv

class PolicyNetwork(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(PolicyNetwork, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, output_dim)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        return self.fc2(x)

def train_policy():
    env = MultiAgentEnv()
    policy = PolicyNetwork(input_dim=env.get_state().size, output_dim=4)  # Assume 4 actions
    optimizer = optim.Adam(policy.parameters(), lr=1e-3)
    num_episodes = 5

    for episode in range(num_episodes):  # Number of episodes
        env.reset(seed=np.random.randint(1, 99))
        state = env.get_state()
        done = False

        while not done:
            state_tensor = torch.tensor(state, dtype=torch.float32)
            action_probs = policy(state_tensor)
            action = torch.argmax(action_probs).item()

            # Perform the action in the environment
            # env.step(action)  # Implement this method to move the robots

            reward = env.calculate_reward()
            optimizer.zero_grad()
            loss = -torch.log(action_probs[action]) * reward
            loss.backward()
            optimizer.step()

            state = env.get_state()

train_policy()
