# -------------------------------
# DDPG Algorithm
# Author:  Kim Young Gi(HCIR Lab.)
# Date: 2021. 1. 28
# reference: https://github.com/philtabor/Youtube-Code-Repository/blob/master/ReinforcementLearning/PolicyGradient/DDPG/pytorch/lunar-lander/ddpg_torch.py
# -------------------------------
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import os

np.random.seed(14512281)

class OrnsteinUhlenbeck(object):
    """
    Ornstein-Uhlenbeck Process
    return: exploration noise
    """
    def __init__(self, mu=0, theta=0.15, sigma=0.2, dt=1e-2, x0=None):
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.dt = dt
        self.x0 = x0
        self.x_prev = self.x0 if self.x0 is not None else np.zeros_like(self.mu)
    
    def __call__(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + self.sigma * np.sqrt(self.dt) * np.random.normal(size=self.mu.shape)
        self.x_prev = x
        return x 

"""
ReplayBuffer
"""
import collections
import random

class ReplayBuffer(object):
    """
    input: 
        Transition: state, action, reward, next_state
    return: Transition sample in buffer
    """
    def __init__(self, buffer_size=1000000, batch_size=64):
        self.buffer_size = buffer_size
        self.batch_size = batch_size
        self.memory = collections.deque(maxlen=self.buffer_size)
    
    def __len__(self):
        return len(self.memory)

    def store(self, state, action, reward, next_state, done):
        self.memory.append([state, action, reward, next_state, done])

    def sample(self):
        state_sample = []
        action_sample = []
        reward_sample = []
        next_state_sample = []
        done_sample = []

        batch_sample = random.sample(self.memory, self.batch_size)
        
        for transition_ in batch_sample:
            _state, _action, _reward, _next_state, _done = transition_
            state_sample.append(_state)
            action_sample.append(_action)
            reward_sample.append(_reward)
            next_state_sample.append(_next_state)
            done_sample.append(_done)

        return state_sample, action_sample, reward_sample, next_state_sample, done_sample

"""
- Network
- actor-critic method, 'soft' target update
- actor-critic: Q, Mu  -> parameter: PI
- Target actor-critic: Q', Mu' -> copy of the actor-critic(Q, Mu) -> parameter: PI'
    - Target Network update: PI' <- tau*PI + (1-tau)*PI' (tau << 1)
"""

DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cuda:1')

class Actor(nn.Module): # Policy
    """
    input: state
    return: tanh(action)
    optimization: Adam
    activation: ReLU(All hidden layer)
    2 hidden layer: 400-300
    final layer weights and biases initialized from a uniform distribution -3x1e-3, 3x1e-3: -0.003, 0.003
    """
    def __init__(self, obs_dim, action_dim, hidden1=400, hidden2=300, learning_rate=0.0001, init_weight=0.003):
        super(Actor, self).__init__()
        self.lr = learning_rate
        self.init_w = init_weight
        
        self.fc1 = nn.Linear(obs_dim, hidden1)
        self.bn1 = nn.LayerNorm(hidden1)
        self.fc2 = nn.Linear(hidden1, hidden2)
        self.bn2 = nn.LayerNorm(hidden2)
        self.fc3 = nn.Linear(hidden2, action_dim)

        fc1_v = self.fanin_init(self.fc1.weight.data.size()[0])
        fc2_v = self.fanin_init(self.fc2.weight.data.size()[0])
        nn.init.uniform_(self.fc1.weight, -fc1_v, fc1_v)
        nn.init.uniform_(self.fc1.bias, -fc1_v, fc1_v)
        nn.init.uniform_(self.fc2.weight, -fc2_v, fc2_v)
        nn.init.uniform_(self.fc2.bias, -fc2_v, fc2_v)
        nn.init.uniform_(self.fc3.weight, -self.init_w, self.init_w)
        nn.init.uniform_(self.fc3.bias, -self.init_w, self.init_w)

        self.optimizer = torch.optim.Adam(self.parameters(), lr=self.lr)
    
    def fanin_init(self, size):
        value = 1. / np.sqrt(size)
        return value

    def forward(self, obs):
        mu = self.fc1(obs)
        mu = self.bn1(mu)
        mu = F.relu(mu)
        mu = self.fc2(mu)
        mu = self.bn2(mu)
        mu = F.relu(mu)
        mu = self.fc3(mu)
        mu = torch.tanh(mu)
        return mu

class Critic(nn.Module): # Action-value Function(Q-function)
    """
    input: State, Action
    return: action-value
    optimization: Adam
    activation: ReLU(All hidden layer)
    2 hidden layer: 400-300
    Action were not included until the 2nd hidden layer of Q
    final layer weights and biases initialized from a uniform distribution -3x1e-4, 3x1e-4: -0.0003, 0.0003
    """
    def __init__(self, obs_dim, action_dim, hidden1=400, hidden2=300, learning_rate=0.001, init_weight=0.0003):
        super(Critic, self).__init__()
        self.lr = learning_rate
        self.init_w = init_weight

        self.fc1 = nn.Linear(obs_dim, hidden1)
        self.bn1 = nn.LayerNorm(hidden1)
        self.fc2 = nn.Linear(hidden1+action_dim, hidden2)
        self.bn2 = nn.LayerNorm(hidden2)
        self.fc3 = nn.Linear(hidden2, 1)
        
        fc1_v = self.fanin_init(self.fc1.weight.data.size()[0])
        fc2_v = self.fanin_init(self.fc2.weight.data.size()[0])
        nn.init.uniform_(self.fc1.weight, -fc1_v, fc1_v)
        nn.init.uniform_(self.fc1.bias, -fc1_v, fc1_v)
        nn.init.uniform_(self.fc2.weight, -fc2_v, fc2_v)
        nn.init.uniform_(self.fc2.bias, -fc2_v, fc2_v)
        nn.init.uniform_(self.fc3.weight, -self.init_w, self.init_w)
        nn.init.uniform_(self.fc3.bias, -self.init_w, self.init_w)

        self.optimizer = torch.optim.Adam(self.parameters(), lr=self.lr)
    
    def fanin_init(self, size):
        value = 1. / np.sqrt(size)
        return value
    
    def forward(self, obs, action):
        value = self.fc1(obs)
        value = self.bn1(value)
        value = F.relu(value)
        action_value = self.fc2(torch.cat([value,action],1))
        action_value = self.bn2(action_value)
        action_value = F.relu(action_value)
        action_value = self.fc3(action_value)
        return action_value

"""
DDPG Algorithm
"""
class DDPGagent(object):
    def __init__(self, tau=0.001, gamma=0.99,
                    obs_dim=18, action_dim=6, action_bound=1.57, 
                    hidden1=400, hidden2=300, actor_lr=0.0001, critic_lr=0.001,
                    buffer_size=1000000, batch_size=64,
                    theta=0.15, sigma=0.2, dt=1e-2, x0=None, checkpoint_path=None):
        self.actor_checkpoint = os.path.join(checkpoint_path, 'actor.pth')
        self.actor_target_checkpoint = os.path.join(checkpoint_path, 'actor_target.pth')
        self.critic_checkpoint = os.path.join(checkpoint_path, 'critic.pth')
        self.critic_target_checkpoint = os.path.join(checkpoint_path, 'critic_target.pth')
        
        self.tau = tau
        self.gamma = gamma

        self.actor_lr = actor_lr
        self.critic_lr = critic_lr
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.action_bound = action_bound

        self.hidden_1 = hidden1
        self.hidden_2 = hidden2
        
        self.buffer_size = buffer_size
        self.batch_size = batch_size

        self.mu = np.zeros(action_dim)
        self.theta = theta
        self.sigma = sigma
        self.dt = dt
        self.x0 = x0
    
        self.memory = ReplayBuffer(buffer_size=self.buffer_size, batch_size=self.batch_size)
        
        self.actor = Actor(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                            hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.actor_lr).to(DEVICE)
        self.actor_target = Actor(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                                    hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.actor_lr).to(DEVICE)
        self.critic = Critic(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                                hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.critic_lr).to(DEVICE)
        self.critic_target = Critic(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                                        hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.critic_lr).to(DEVICE)
        
        self.update_network_parameters(tau=1)

        self.noise = OrnsteinUhlenbeck(mu=self.mu, theta=self.theta, sigma=self.sigma, dt=self.dt, x0=self.x0)

    def remember(self, state, action, reward, next_state, done):
        self.memory.store(state, action, reward, next_state, done)

    def getAction(self, observation):
        self.actor.eval()
        obs = torch.tensor(observation, dtype=torch.float).to(DEVICE)
        mu = self.actor(obs).to(DEVICE)
        mu = mu * self.action_bound

        a_t = mu + torch.tensor(self.noise(), dtype=torch.float).to(DEVICE)
        a_t = a_t.cpu().detach().numpy()

        self.actor.train()
        return a_t
    
    def learn(self):
        if len(self.memory) < self.batch_size:
            return
        
        state, action, reward, new_state, done = self.memory.sample()

        reward = torch.tensor(reward, dtype=torch.float).to(DEVICE)
        done = torch.tensor(done).to(DEVICE)
        new_state = torch.tensor(new_state, dtype=torch.float).to(DEVICE)
        action = torch.tensor(action, dtype=torch.float).to(DEVICE)
        state = torch.tensor(state, dtype=torch.float).to(DEVICE)

        self.actor_target.eval()
        self.critic_target.eval()
        self.critic.eval()

        target_actions = self.actor_target(new_state)
        critic_value_ = self.critic_target(new_state, target_actions)
        critic_value = self.critic(state, action)

        target = []
        for j in range(self.batch_size):
            target.append(reward[j] + self.gamma*critic_value_[j]*(1-done[j]))
        target = torch.tensor(target).to(DEVICE)
        target = target.view(self.batch_size, 1)

        self.critic.train()
        self.critic.optimizer.zero_grad()
        critic_loss = F.mse_loss(target, critic_value)
        critic_loss.backward()
        self.critic.optimizer.step()

        self.critic.eval()
        self.actor.optimizer.zero_grad()
        mu = self.actor(state)
        self.actor.train()
        actor_loss = -self.critic(state, mu)
        actor_loss = torch.mean(actor_loss)
        actor_loss.backward()
        self.actor.optimizer.step()

        self.update_network_parameters()

    def update_network_parameters(self, tau=None):
        if tau == None:
            tau = self.tau

        actor_params = self.actor.named_parameters()
        critic_params = self.critic.named_parameters()
        actor_target_params = self.actor_target.named_parameters()
        critic_target_params = self.critic_target.named_parameters()

        critic_state_dict = dict(critic_params)
        critic_target_dict = dict(critic_target_params)
        actor_state_dict = dict(actor_params)
        actor_target_dict = dict(actor_target_params)

        for name in critic_state_dict:
            critic_state_dict[name] = tau*critic_state_dict[name].clone() + \
                                      (1-tau)*critic_target_dict[name].clone()

        self.critic_target.load_state_dict(critic_state_dict)

        for name in actor_state_dict:
            actor_state_dict[name] = tau*actor_state_dict[name].clone() + \
                                      (1-tau)*actor_target_dict[name].clone()
        self.actor_target.load_state_dict(actor_state_dict)

    def save_models(self):
        print('... saving checkpoint ...')
        torch.save(self.actor.state_dict(), self.actor_checkpoint)
        torch.save(self.actor_target.state_dict(), self.actor_target_checkpoint)
        torch.save(self.critic.state_dict(), self.critic_checkpoint)
        torch.save(self.critic_target.state_dict(), self.critic_target_checkpoint)

    def load_models(self):
        print('... loading checkpoint ...')
        self.actor.load_state_dict(torch.load(self.actor_checkpoint))
        self.actor_target.load_state_dict(torch.load(self.actor_target_checkpoint))
        self.critic.load_state_dict(torch.load(self.critic_checkpoint))
        self.critic_target.load_state_dict(torch.load(self.critic_target_checkpoint))


if __name__ == "__main__":
    pass 