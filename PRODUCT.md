# Haggatrons product brief

## The idea

Haggatrons is a team of small, battery-powered exploration robots. They enter an unfamiliar indoor area, look around independently, and share useful observations. A coordinator assigns distinct exploration goals and maintains a shared picture of what the team has found. Each robot decides how to carry out its assigned goal with its own sensors and motor controller. A human supervises the mission and can stop it.

The planned kit contains four robots. A meaningful first demonstration can use one physical robot and additional simulated workers; the intended team demonstration uses at least two independently operating robots.

## Hackathon fit

The [Collaborative Agent Hackathon @ Stanford](https://luma.com/flwrlabs-bamu) calls for an open-source agent or multi-agent system built with Flower, with safe, human-supervised collaboration central to the solution. Haggatrons uses Flower for the exchange between robot workers and the coordinator. Shared observations, distinct assignments, and explicit human control should be visible in the demo, not just described in slides.

## Operating model

1. A robot captures a camera frame and IMU reading, then forms a local observation with its own identity, timestamp, confidence, and possible hazards.
2. A visual model can describe the frame and suggest a bounded next action. OpenAI `gpt-6-luna` is the selected visual model. Its output is a proposal, never a direct motor command.
3. The Flower coordinator combines reports into a shared exploration state, avoids duplicate assignments, and sends each robot a goal or stop instruction.
4. The robot checks its local sensor state and safety limits before executing a short movement. It reports what actually happened, including failures and uncertainty.
5. The human can start, pause, or stop the mission and inspect observations, assignments, and actions.

The Mac may host the Flower coordinator and OpenAI calls. Robots must operate on battery **without a physical connection to the Mac** during the mission. The wireless connection mechanism is an implementation choice; the product requirement is reliable two-way communication while untethered.

## Hardware and roles

The bill of materials plans four ESP32 camera boards, four 3.7 V LiPo batteries, eight N20 DC motors, and four TB6612FNG dual motor drivers. Each robot therefore has one camera board, one battery, two motors, and one driver. The tested board is a Meshnology W11 ESP32-S3 with a camera and QMI8658 IMU.

The ESP32 handles sensing, local motor actuation, and the immediate stop behavior. The Mac handles coordination, run logs, and visual model requests. The IMU measures acceleration and rotation rate; it does not provide a reliable global position on its own. Camera-only free-space judgments are uncertain, so short moves and frequent re-observation are required.

## First useful demo

The minimum convincing physical demo is:

- One battery-powered robot sends a fresh image and IMU sample to the Mac while unplugged.
- Its visual worker produces an observation and a bounded action proposal.
- The coordinator displays that observation, assigns a goal, and records why it chose it.
- The robot executes a short approved motor action and reports completion or a stop condition.
- A visible human stop interrupts movement, and communication loss causes the robot to stop locally.

The team demo adds a second physical robot, distinct goals, shared discoveries, and collision or conflict avoidance. Simulation remains useful for repeatable checks but does not count as physical validation.

## Safety and quality requirements

- Keep movement commands short and bounded by local firmware. On timeout, lost communication, invalid input, or a stop request, de-energize the driver.
- Keep the visual model advisory. Validate its output and apply safety checks before any movement.
- Do not claim a map cell is traversable from one image alone. Represent uncertainty and re-check after moving.
- Log each robot's observation, assignment, proposed action, approved action, and actual outcome with timestamps.
- Keep API keys and local access credentials out of Git and demo output. Minimize paid visual-model calls during development by using saved frames and capture-only tests.
- Keep a fully repeatable simulation path for the Flower collaboration logic.

## Current scope

The repository currently has a two-worker Flower grid simulation, a camera/IMU capture path, and a validated single-image Luna observer. The real camera observation does not yet update the shared map or drive motors. Wireless camera firmware has been flashed and checked over USB; an unplugged wireless capture and physical motor motion are still pending. See [CONTEXT.md](CONTEXT.md) for exact status and next steps.
