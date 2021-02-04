# -------------------------------
# PPO Algorithm
# Author:  Kim Young Gi(HCIR Lab.)
# Date: 2021. 1. 26
# reference: https://github.com/nikhilbarhate99/PPO-PyTorch/blob/master/PPO_continuous.py
# -------------------------------

import torch
import torch.nn as nn
from torch.distributions import MultivariateNormal
import gym
import numpy as np
import yaml
import os

DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
    
    def clear_memory(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]

class ActorCritic(nn.Module):
    def __init__(self, obs_dim, action_dim, action_std, hidden1, hidden2):
        super(ActorCritic, self).__init__()
        # action mean range -1 to 1
        self.actor =  nn.Sequential(
                nn.Linear(obs_dim, hidden1),
                nn.Tanh(),
                nn.Linear(hidden1, hidden2),
                nn.Tanh(),
                nn.Linear(hidden2, action_dim),
                nn.Tanh()
                )
        # critic
        self.critic = nn.Sequential(
                nn.Linear(obs_dim, hidden1),
                nn.Tanh(),
                nn.Linear(hidden1, hidden2),
                nn.Tanh(),
                nn.Linear(hidden2, 1)
                )
        self.action_var = torch.full((action_dim,), action_std).to(DEVICE)
        
    def forward(self):
        raise NotImplementedError
    
    def act(self, state, memory, action_bound):
        action_mean = self.actor(state) * action_bound
        cov_mat = torch.diag(self.action_var).to(DEVICE)
        
        dist = MultivariateNormal(action_mean, cov_mat)
        action = dist.sample()
        action_logprob = dist.log_prob(action)
        
        memory.states.append(state)
        memory.actions.append(action)
        memory.logprobs.append(action_logprob)
        
        return action.detach()
    
    def evaluate(self, state, action):   
        action_mean = self.actor(state)
        
        action_var = self.action_var.expand_as(action_mean)
        cov_mat = torch.diag_embed(action_var).to(DEVICE)
        
        dist = MultivariateNormal(action_mean, cov_mat)
        
        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()
        state_value = self.critic(state)
        
        return action_logprobs, torch.squeeze(state_value), dist_entropy

class PPOagent:
    def __init__(self, obs_dim, action_dim, action_std, action_bound, hidden1, hidden2, lr, betas, gamma, n_epochs, clipping, checkpoint_path=None):
        self.policy_checkpoint = os.path.join(checkpoint_path, 'ppo_policy.pth')
        self.action_bound = action_bound

        self.lr = lr
        self.betas = betas
        self.gamma = gamma
        self.clipping = clipping
        self.n_epochs = n_epochs
        
        self.policy = ActorCritic(obs_dim, action_dim, action_std, hidden1, hidden2).to(DEVICE)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr, betas=betas)
        
        self.policy_old = ActorCritic(obs_dim, action_dim, action_std, hidden1, hidden2).to(DEVICE)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()

        self.memory = Memory()
    
    def memorize(self, reward, done):
        self.memory.rewards.append(reward)
        self.memory.is_terminals.append(done)

    def getAction(self, state):
        state = torch.FloatTensor(state.reshape(1, -1)).to(DEVICE)
        return self.policy_old.act(state, self.memory, self.action_bound).cpu().data.numpy().flatten()
    
    def update(self):
        # Monte Carlo estimate of rewards:
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.memory.rewards), reversed(self.memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        
        # Normalizing the rewards:
        rewards = torch.FloatTensor(rewards).to(DEVICE)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)
        
        # convert list to tensor
        old_states = torch.squeeze(torch.stack(self.memory.states).to(DEVICE), 1).detach()
        old_actions = torch.squeeze(torch.stack(self.memory.actions).to(DEVICE), 1).detach()
        old_logprobs = torch.squeeze(torch.stack(self.memory.logprobs), 1).to(DEVICE).detach()
        
        # Optimize policy for Num. epochs:
        for _ in range(self.n_epochs):
            # Evaluating old actions and values :
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            
            # Finding the ratio (pi_theta / pi_theta__old):
            ratios = torch.exp(logprobs - old_logprobs.detach())

            # Finding Surrogate Loss:
            advantages = rewards - state_values.detach()   
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.clipping, 1+self.clipping) * advantages
            loss = -torch.min(surr1, surr2) + 0.5*self.MseLoss(state_values, rewards) - 0.01*dist_entropy
            
            # take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
            
        # Copy new weights into old policy:
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.memory.clear_memory()
    
    def save_models(self):
        print('... saving checkpoint ...')
        torch.save(self.policy.state_dict(), self.policy_checkpoint)
        
    def load_models(self):
        print('... loading checkpoint ...')
        self.policy.load_state_dict(torch.load(self.policy_checkpoint))
        