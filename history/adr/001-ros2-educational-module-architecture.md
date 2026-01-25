# ADR 001: ROS 2 Educational Module Architecture

## Status
Accepted

## Context
We needed to design an educational module for teaching ROS 2 concepts (Nodes, Topics, Services, and URDF) that would be:
- Accessible to students with basic programming knowledge
- Compatible with the Jetson Orin Nano hardware platform
- Follow the project's constitution requirements for technical accuracy and reproducibility
- Provide hands-on lab exercises with practical examples

## Decision
We chose to implement a multi-component educational module with:

1. **Technology Stack**:
   - ROS 2 Humble Hawksbill (LTS version for stability)
   - Python 3.8+ as the primary language for examples
   - rclpy as the ROS 2 client library
   - Standard ROS 2 message和服务 types with custom extensions

2. **Project Structure**:
   - Code examples in `src/ros2_examples/` with clear directory organization
   - Educational content in `docs/ros2_nervous_system/` with separate files for concepts, architecture, and lab exercises
   - Integration tests in `tests/integration/` for verification
   - Comprehensive documentation following the "Concepts → Architecture → Code → Lab → Quiz" structure

3. **Communication Patterns**:
   - Publisher/subscriber for continuous data streams
   - Services for request/response interactions
   - Quality of Service (QoS) settings appropriate for educational use
   - URDF for robot modeling and description

## Alternatives Considered

### Technology Stack Alternatives:
- **ROS 1 vs ROS 2**: Chose ROS 2 for its improved security, distributed architecture, and long-term support
- **C++ vs Python**: Chose Python for educational accessibility despite C++ performance advantages
- **Rolling vs LTS**: Chose Humble LTS for stability in educational setting

### Architecture Alternatives:
- **Monolithic vs Modular**: Chose modular design for easier learning progression
- **Simulation-only vs Real Hardware**: Designed for both with Jetson Orin Nano as target platform
- **Single Language vs Multi-Language**: Focused on Python for consistency in educational context

## Consequences

### Positive:
- Students can start with simple examples and progress to complex concepts
- Code is accessible to students with basic programming knowledge
- Architecture follows ROS 2 best practices
- Compatible with Jetson Orin Nano hardware constraints
- Comprehensive testing ensures reproducibility

### Negative:
- Python may not represent performance characteristics of production systems
- Educational focus may not reflect all production considerations
- Single-language approach doesn't demonstrate multi-language ROS 2 capabilities

## Links
- Related to: specs/001-ros2-nervous-system/spec.md
- Implements: Constitution requirements for technical accuracy and reproducibility
- Follow-up: Additional ADRs for advanced ROS 2 concepts