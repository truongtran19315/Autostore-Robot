# Import the required modules
import gym
import pygame
import numpy as np

pygame.init()
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption("MountainCar-v0")

# Create a font object
font = pygame.font.SysFont("Arial", 20)
# Create a clock object to control the frame rate
clock = pygame.time.Clock()

env = gym.make("MountainCar-v0", render_mode="rgb_array")
observation = env.reset()
total_reward = 0
done = False
while not done:
    keys = pygame.key.get_pressed()
    action = 1
    if keys[pygame.K_a]:
        action = 2
    if keys[pygame.K_d]:
        action = 0
    if keys[pygame.K_s]:
        action = 1
    print(f"action : {action}")
    observation, reward, done, info, _ = env.step(action)
    total_reward += reward

    #! Chuyển đổi mảng numpy thành bề mặt pygame, vì env.step() trả về mảng numpy chứ không phải là bề mặt 
    surf = pygame.surfarray.make_surface(env.render())
    surf = pygame.transform.rotate(surf, -90)   #! Xoay bề mặt pygame 90 độ để nằm ngang 
    screen.blit(surf, (0, 0))   # Render the environment on the screen

    text = font.render(f"Total reward: {total_reward:.2f}", True, (0, 0, 0))
    screen.blit(text, (200, 10))
    text = font.render(f"Position: {observation[0]:.2f}, Velocity: {observation[1]:.2f}", True, (0, 0, 0))
    screen.blit(text, (200, 50))
    
    pygame.display.flip()   # Update the display
    clock.tick(60)          # Limit the frame rate to 60 FPS
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
            
env.close()
pygame.quit()