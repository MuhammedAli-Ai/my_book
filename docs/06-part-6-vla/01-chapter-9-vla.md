---
id: chapter-9-vla-cognitive-planning
title: "Chapter 9: VLA & Cognitive Planning"
sidebar_label: "9. VLA & Cognitive Planning"
---

# Vision-Language-Action (VLA) and Cognitive Planning

## 9.1 The Convergence: LLMs + Robotics

Large Language Models (LLMs) like GPT-4 contain vast world knowledge. They know that "sponges clean plates" and "apples are edible". **Robotics** traditionally knew nothing about the world.

**VLA (Vision-Language-Action)** models bridge this. They take image + text as input and output **Action Tokens** (robot commands).

### Examples
*   **Google RT-2**: "Pick up the extinct animal" -> Robot picks up the dinosaur toy.
*   **Tesla Optimus**: End-to-end neural networks from video to motor torque.

## 9.2 Voice-to-Action: OpenAI Whisper

To interact naturally, we need speech.
**OpenAI Whisper** is a robust Automatic Speech Recognition (ASR) model.
1.  **Input**: Audio waveform from robot microphone.
2.  **Output**: Text string "Go to the kitchen".
3.  **Pipeline**: `Audio -> Whisper -> Text -> LLM -> Planner`.

## 9.3 Cognitive Planning with LLMs

We use LLMs as the "High-Level Planner".

### Prompt Engineering for Robots
We give the LLM a system prompt defining the robot's capabilities:
> "You are a robot helper. You have functions: `pick(obj)`, `move_to(loc)`, `say(text)`. User: 'I'm thirsty'."

**LLM Output**:
```json
[
  {"action": "move_to", "arg": "kitchen"},
  {"action": "pick", "arg": "water_bottle"},
  {"action": "move_to", "arg": "user"},
  {"action": "say", "arg": "Here is some water."}
]
```

This JSON is parsed by a Python node and executed via ROS 2 Actions.

## 9.4 Grounding

**Grounding** is the problem of linking text ("the red cup") to physical coordinates (x=1.2, y=0.5).
*   **Open Vocabulary Detectors (OWL-ViT)**: Find "red cup" in image -> Bounding Box (pixels).
*   **Depth Camera**: Convert Pixels -> 3D Point.
*   **TF**: Convert Camera Frame -> Map Frame.

Now the robot has grounded the abstract concept "red cup" into physical reality.
