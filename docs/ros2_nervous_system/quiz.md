# ROS 2 Robotic Nervous System - Assessment Quiz

This quiz assesses understanding of the ROS 2 fundamental concepts (Nodes, Topics, Services, URDF) as outlined in the success criteria.

## Quiz Questions

### Section 1: ROS 2 Architecture Concepts (Multiple Choice)

**Q1.** What is a ROS 2 Node?
- A) A message type used for communication
- B) A process that performs computation and communicates with other nodes
- C) A hardware component on the Jetson Orin Nano
- D) A type of robot joint in URDF

**Q2.** Which communication pattern is asynchronous in ROS 2?
- A) Services
- B) Actions
- C) Topics (publish/subscribe)
- D) Parameters

**Q3.** What does URDF stand for?
- A) Universal Robot Development Framework
- B) Unified Robot Description Format
- C) Universal Robotics Data Format
- D) Unified Robot Design Framework

**Q4.** In the publisher/subscriber pattern:
- A) The publisher waits for a response from the subscriber
- B) The subscriber sends requests to the publisher
- C) The publisher sends messages without waiting for a response
- D) Both publisher and subscriber must be active simultaneously

### Section 2: Technical Understanding (Short Answer)

**Q5.** Explain the difference between ROS 2 Services and Topics in terms of communication pattern and use cases.

**Q6.** What are the three main components of a URDF robot model? Briefly describe each.

**Q7.** What is Quality of Service (QoS) in ROS 2 and why is it important?

### Section 3: Practical Application (Scenario-based)

**Q8.** You are designing a robot with a camera that streams images continuously to a processing node. Which ROS 2 communication pattern would you use and why? What QoS settings would be appropriate?

**Q9.** You need to send a command to move a robot's arm to a specific position and wait for confirmation. Which ROS 2 communication pattern would be most appropriate and why?

**Q10.** When deploying ROS 2 nodes on a Jetson Orin Nano, what considerations should you make regarding computational resources?

## Answer Key

### Section 1 Answers:
1. **B** - A process that performs computation and communicates with other nodes
2. **C** - Topics (publish/subscribe)
3. **B** - Unified Robot Description Format
4. **C** - The publisher sends messages without waiting for a response

### Section 2 Answers:
5. **Services vs Topics:**
   - Services: Synchronous request/response pattern, one-to-one communication, used for discrete actions that require a response
   - Topics: Asynchronous publish/subscribe pattern, one-to-many or many-to-many communication, used for continuous data streams

6. **URDF Components:**
   - **Links**: Rigid body elements that represent parts of the robot
   - **Joints**: Connections between links that define how they move relative to each other
   - **Visual/Collision**: Properties that define how the robot appears and interacts physically

7. **QoS (Quality of Service):**
   - QoS settings control how messages are delivered between nodes
   - Important for ensuring appropriate behavior for different types of communication
   - Includes reliability (reliable vs best effort), durability (volatile vs transient), and history settings

### Section 3 Answers:
8. **Camera streaming:** Use Topics (publish/subscribe) because it's a continuous data stream where the publisher doesn't need to wait for responses. Appropriate QoS would be BEST_EFFORT reliability and appropriate depth for buffering.

9. **Arm movement:** Use Services because you need to send a request and wait for confirmation that the action was completed. Service provides the synchronous request/response pattern needed.

10. **Jetson considerations:** Monitor CPU/GPU usage, manage memory efficiently, consider thermal constraints, optimize algorithms for ARM architecture, and implement appropriate error handling for resource limitations.

## Scoring Rubric

- **Section 1 (Multiple Choice)**: 1 point each (4 points total)
- **Section 2 (Short Answer)**: 2 points each (6 points total)
- **Section 3 (Scenario-based)**: 3 points each (9 points total)
- **Total**: 19 points

### Passing Criteria
- Minimum 80% (15/19) to pass the assessment
- Students should demonstrate understanding of core ROS 2 concepts
- Practical application of knowledge to real-world scenarios

## Learning Outcomes Assessment

This quiz verifies that students meet the success criteria from the specification:
- ✅ Students demonstrate understanding of ROS 2 Graph Architecture
- ✅ Students can successfully execute a "Hello World" Publisher/Subscriber lab exercise
- ✅ Students achieve 80% or higher on assessments covering ROS 2 fundamental concepts