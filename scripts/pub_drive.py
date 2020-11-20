#!/usr/bin/env python
import rospy
import numpy as np
import gym, os
import gym_pathplan
from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam
from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess
import matplotlib.pyplot as plt
import math
import tensorflow as tf
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


global observe_data
observe_data = np.array([0,0,0,0,0,0,0,0,0,0,0])

def callback_observe(data):
    global observe_data
    observe_data = np.array(data.data)
    #print(observe_data)
    #print(agent.forward(observe_data))
    for i in range(9): #laser num!!!!!!!!!!!!!
        if observe_data[i] < 0.2:
            observe_data[i] = 0.2

ENV_NAME = 'Simple-v0'

# Get the environment and extract the number of actions.
env = gym.make(ENV_NAME)
env.reset()
np.random.seed(123)
env.seed(123)
assert len(env.action_space.shape) == 1
nb_actions = env.action_space.shape[0]

# Next, we build a very simple model.
actor = Sequential()
actor.add(Flatten(input_shape=(1,) + env.observation_space.shape))
actor.add(Dense(512))
actor.add(Activation('relu'))
actor.add(Dense(256))
actor.add(Activation('relu'))
actor.add(Dense(128))
actor.add(Activation('relu'))
actor.add(Dense(64))
actor.add(Activation('relu'))
actor.add(Dense(nb_actions))
actor.add(Activation('tanh'))
#print(actor.summary())

action_input = Input(shape=(nb_actions,), name='action_input')
observation_input = Input(shape=(1,) + env.observation_space.shape, name='observation_input')
flattened_observation = Flatten()(observation_input)
x = Concatenate()([action_input, flattened_observation])
x = Dense(512)(x)
x = Activation('relu')(x)
x = Dense(256)(x)
x = Activation('relu')(x)
x = Dense(128)(x)
x = Activation('relu')(x)
x = Dense(64)(x)
x = Activation('relu')(x)
x = Dense(1)(x)
x = Activation('linear')(x)
critic = Model(inputs=[action_input, observation_input], outputs=x)
#print(critic.summary())

memory = SequentialMemory(limit=100000, window_length=1)
random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)

agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
                  memory=memory, nb_steps_warmup_critic=100, nb_steps_warmup_actor=100,
                  random_process=random_process, gamma=.99, target_model_update=1e-3)
agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])

try:
    agent.load_weights(os.path.dirname(__file__) + '/weights/trained_weight/ddpg_{}_weights.h5f'.format(ENV_NAME))
    print("find weights-file")
except:
    print("not found weights-file")

rospy.init_node('pub_drive', anonymous=True)
rospy.Subscriber("/observe", Float32MultiArray, callback_observe)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10) # 5hz

fig, ax = plt.subplots(1, 1)
ax.set_ylim(math.radians(-50), math.radians(50))
x = []
y = []
step_count = 0

while not rospy.is_shutdown():
    predict_action = agent.forward(observe_data)
    pub_vel = Twist()
    if predict_action[0] > 0.8:
        pub_vel.linear.x = 0.8
    elif predict_action[0] < -0.8:
        pub_vel.linear.x = -0.8
    else:
        pub_vel.linear.x = predict_action[0]
    if predict_action[1] > 0.8:
        pub_vel.angular.z = 0.8
    elif predict_action[1] < -0.8:
        pub_vel.angular.z = -0.8
    else:
        pub_vel.angular.z = predict_action[1]

    pub_vel.linear.x = pub_vel.linear.x*0.5
    pub_vel.angular.z = pub_vel.angular.z * 0.5

    pub.publish(pub_vel)
    #print("\r"+str(pub_vel) + "                  ",end="")
    print(str(pub_vel))
    #print(predict_action)
    
    step_count += 1
    x.append(step_count)
    y.append(pub_vel.angular.z*4)
    
    if(len(x)>150):
        x.pop(0)
        y.pop(0)
    plt.xlim(x[0],x[0]+150)

    line, = ax.plot(x, y, color='blue')

    plt.pause(0.01)
    # グラフをクリア
    line.remove()


    r.sleep()

rospy.spin()
