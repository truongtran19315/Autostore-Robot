import gym


# Tao bien moi truong
env = gym.make("MountainCar-v0", render_mode= "human")

env.reset()     

# lay state hien tai sau khi khoi tao
print(env.state)

# lay so acion ma xe co the thuc hien duoc 
print(env.action_space.n)

print(env.action_space)
# Sample a random action from the entire action space
random_action = env.action_space.sample()

#Take the action and get the new observation space
new_obs, reward, done, noinfo,info = env.step(random_action)
print("The new observation is {}".format(new_obs))

while True:
    new_state, reward, done, no_info, info = env.step(action=2)
    print("new_state = {}, reward = {}, is_done = {}, info = {}".format(new_state, reward, done, info))
    
    env.render()
    
env.close()
