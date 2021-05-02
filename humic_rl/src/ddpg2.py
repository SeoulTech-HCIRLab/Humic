# -------------------------------
# DDPG Algorithm
# Author:  Kim Young Gi(HCIR Lab.)
# reference: https://github.com/philtabor/Youtube-Code-Repository/blob/master/ReinforcementLearning/PolicyGradient/DDPG/pytorch/lunar-lander/ddpg_torch.py
# -------------------------------
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import os

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
from collections import deque
import random

class ReplayBuffer(object):
    """
    input: 
        Transition: state, action, reward, next_state
    return: Transition sample in buffer
    """
    def __init__(self, buffer_size=1000000, batch_size=64, seed=123456):
        self.seed = random.seed(seed)
        self.buffer_size = buffer_size
        self.batch_size = batch_size
        self.memory = deque(maxlen=self.buffer_size)
    
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

# https://github.com/dranaju/project/blob/master/src/ddpg_stage_1.py
# Virtual-to-real Deep Reinforcement Learning: Continuous Control of Mobile Robots for Mapless Navigation

class Actor(nn.Module): # Policy
    """
    input: state
    return: tanh(action)
    optimization: Adam
    activation: ReLU(All hidden layer)
    2 hidden layer: 400-300
    final layer weights and biases initialized from a uniform distribution -3e-3, 3e-3
    """
    def __init__(self, obs_dim, action_dim, hidden1=400, hidden2=300, learning_rate=0.0001, init_weight=0.003, seed=123456):
        super(Actor, self).__init__()
        self.seed = torch.manual_seed(seed)
        self.obs_dim = obs_dim
        self.lr = learning_rate
        
        self.fc1 = nn.Linear(obs_dim, hidden2)
        nn.init.xavier_uniform_(self.fc1.weight)
        self.fc1.bias.data.fill_(0.01)
        
        self.fc2 = nn.Linear(hidden2, hidden2)
        nn.init.xavier_uniform_(self.fc2.weight)
        self.fc2.bias.data.fill_(0.01)

        self.fc3 = nn.Linear(hidden2, hidden2)
        nn.init.xavier_uniform_(self.fc3.weight)
        self.fc3.bias.data.fill_(0.01)
        
        self.fc = nn.Linear(hidden2, action_dim)
        nn.init.xavier_uniform_(self.fc.weight)
        self.fc.bias.data.fill_(0.01)        
        
        self.optimizer = torch.optim.Adam(self.parameters(), lr=self.lr)

    def forward(self, obs):
        mu = torch.relu(self.fc1(obs))
        mu = torch.relu(self.fc2(mu))
        mu = torch.relu(self.fc3(mu))
        mu = self.fc(mu)
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
    final layer weights and biases initialized from a uniform distribution -3e-4, 3e-4
    """
    def __init__(self, obs_dim, action_dim, hidden1=400, hidden2=300, learning_rate=0.001, init_weight=0.0003, seed=123456):
        super(Critic, self).__init__()
        self.seed = torch.manual_seed(seed)
        self.lr = learning_rate

        self.fc1s = nn.Linear(obs_dim, hidden1)        
        nn.init.xavier_uniform_(self.fc1s.weight)
        self.fc1s.bias.data.fill_(0.01)
        
        self.fc2 = nn.Linear(hidden1+action_dim, hidden2)
        nn.init.xavier_uniform_(self.fc2.weight)
        self.fc2.bias.data.fill_(0.01)

        self.fc3 = nn.Linear(hidden2, hidden2)
        nn.init.xavier_uniform_(self.fc3.weight)
        self.fc3.bias.data.fill_(0.01)
        
        self.fc = nn.Linear(hidden2, 1)
        nn.init.xavier_uniform_(self.fc.weight)
        self.fc.bias.data.fill_(0.01)
        
        self.optimizer = torch.optim.Adam(self.parameters(), lr=self.lr, weight_decay=0.01)
    
    def forward(self, obs, action):
        xs = torch.relu(self.fc1s(obs))
        x = torch.relu(self.fc2(torch.cat((xs,action), dim=1)))
        x = torch.relu(self.fc3(x))
        action_value = self.fc(x)
        return action_value

"""
DDPG Algorithm
"""
class DDPGagent(object):
    def __init__(self, params, obs_dim, action_dim, x0=None, checkpoint_path=None):
        self.checkpoint_path = checkpoint_path

        self.obs_dim = obs_dim
        self.action_dim = action_dim

        self.tau = params['tau']
        self.gamma = params['gamma']

        self.actor_lr = params['actor_lr']
        self.critic_lr = params['critic_lr']
        
        self.hidden_1 = params['hidden1_size']
        self.hidden_2 = params['hidden2_size']
        
        self.buffer_size = params['buffer_size']
        self.batch_size = params['batch_size']

        self.mu = np.zeros(self.action_dim)
        self.theta = params['theta']
        self.sigma = params['sigma']
        self.dt = params['dt']
        self.x0 = x0

        self.seed = params['seed']
    
        self.memory = ReplayBuffer(buffer_size=self.buffer_size, batch_size=self.batch_size, seed=self.seed)
        
        self.actor = Actor(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                            hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.actor_lr, seed=self.seed).to(DEVICE)
        self.actor_target = Actor(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                                    hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.actor_lr, seed=self.seed).to(DEVICE)
        self.critic = Critic(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                                hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.critic_lr, seed=self.seed).to(DEVICE)
        self.critic_target = Critic(obs_dim=self.obs_dim, action_dim=self.action_dim, 
                                        hidden1=self.hidden_1, hidden2=self.hidden_2, learning_rate=self.critic_lr, seed=self.seed).to(DEVICE)
        
        self.hard_update(self.actor_target, self.actor)
        self.hard_update(self.critic_target, self.critic)

        self.noise = OrnsteinUhlenbeck(self.mu, theta=self.theta, sigma=self.sigma, dt=self.dt, x0=self.x0)

    def remember(self, state, action, reward, next_state, done):
        self.memory.store(state, action, reward, next_state, done)

    def getAction(self, observation):
        self.actor.eval()
        
        obs = torch.tensor(observation, dtype=torch.float).to(DEVICE)
        mu = self.actor(obs).to(DEVICE)
        action_no = torch.tensor(self.noise(), dtype=torch.float).to(DEVICE)
        mu = mu + action_no

        self.actor.train()
        return mu.cpu().detach().numpy()
    
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
            target.append(reward[j] + done[j]*self.gamma*critic_value_[j])
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

        # Target update
        self.soft_update(self.actor_target, self.actor, self.tau)
        self.soft_update(self.critic_target, self.critic, self.tau)

    def soft_update(self, target, source, tau):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

    def hard_update(self, target, source):
        for target_param, param in zip(target.parameters(), source.parameters()):
                target_param.data.copy_(param.data)

    def save_models(self, e):
        print('... saving checkpoint ...')
        actor_checkpoint = os.path.join(self.checkpoint_path, 'actor{}.pth'.format(e))
        critic_checkpoint = os.path.join(self.checkpoint_path, 'critic{}.pth'.format(e))

        torch.save(self.actor.state_dict(), actor_checkpoint)
        torch.save(self.critic.state_dict(), critic_checkpoint)

    def load_models(self, e):
        print('... loading checkpoint ...')
        actor_checkpoint = os.path.join(self.checkpoint_path, 'actor{}.pth'.format(e))
        critic_checkpoint = os.path.join(self.checkpoint_path, 'critic{}.pth'.format(e))

        self.actor.load_state_dict(torch.load(actor_checkpoint))
        self.critic.load_state_dict(torch.load(critic_checkpoint))
        
if __name__ == "__main__":
    pass 