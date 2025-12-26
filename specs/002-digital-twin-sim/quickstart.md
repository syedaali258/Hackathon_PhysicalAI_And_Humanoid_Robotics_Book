# Quickstart Guide: AI/Spec-Driven Book using Docusaurus - Module 2: Digital Twin

**Feature**: Module 2: The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-21

## Overview

This guide provides a quick setup and initial experience for the AI-native book on Physical AI & Humanoid Robotics, specifically for Module 2: The Digital Twin (Gazebo & Unity). Follow these steps to get started with digital twin simulation concepts using Gazebo and Unity.

## Prerequisites

- Node.js 18+ installed
- Python 3.8+ installed
- Basic understanding of command line tools
- Web browser (Chrome, Firefox, or Safari recommended)
- [Optional] Gazebo Garden or Humble Hawksbill installed for local simulation
- [Optional] Unity 2022.3 LTS installed for Unity components

## Setup Steps

### 1. Clone and Initialize the Book

```bash
# Clone the repository
git clone https://github.com/your-org/ai-robotics-book.git
cd ai-robotics-book

# Install frontend dependencies
cd book
npm install

# Install backend dependencies
cd ../api
pip install -r requirements.txt
```

### 2. Start the Development Environment

```bash
# Terminal 1: Start the Docusaurus development server
cd book
npm start

# Terminal 2: Start the FastAPI backend
cd api
uvicorn main:app --reload --port 8000
```

### 3. Access the Book

1. Open your browser to `http://localhost:3000`
2. You'll see the book homepage with Module 2: The Digital Twin (Gazebo & Unity)
3. The RAG chatbot will be available on every page

## First Experience: Module 2

### Navigate to Module 2

1. Click on "Module 2: The Digital Twin (Gazebo & Unity)" in the sidebar
2. Start with Chapter 1: "Physics Simulation in Gazebo (gravity, collisions)"

### Try the Interactive Features

1. **Read the content**: Each chapter includes conceptual explanations with diagrams
2. **Use the chatbot**: Ask questions about the content using the chat widget
   - Example: "Explain the difference between Gazebo and Unity roles in digital twins"
   - The chatbot will respond based only on book content with source citations
3. **Try simulation examples**: Each chapter includes practical examples with Gazebo and Unity
4. **Experiment with interactive components**: Use the simulation visualization tools

### Sample Interaction with RAG Chatbot

```
You: "What are the main differences between Gazebo and Unity in digital twin applications?"
Bot: "In digital twin applications for robotics:

- **Gazebo**: Specializes in accurate physics simulation with realistic gravity, collisions, and material properties. It's ideal for testing robot algorithms in physically accurate environments.

- **Unity**: Excels at visualization and human-robot interaction interfaces. It provides intuitive user experiences for monitoring and controlling simulated robots.

For comprehensive digital twins, both are often used together with Gazebo handling physics simulation and Unity providing visualization and interaction.

Sources: [Module 2, Chapter 1: Physics Simulation in Gazebo], [Module 2, Chapter 3: Human-Robot Interaction in Unity]

Follow-up questions:
- "How do I set up a physics simulation in Gazebo?"
- "What sensors can be simulated in this environment?"
- "How does Unity integrate with Gazebo for digital twins?"
```

## Content Structure

The book is organized as follows:

```
Module 1: The Robotic Nervous System (ROS 2)
├── Chapter 1: ROS 2 Fundamentals: Nodes, Topics, and Services
├── Chapter 2: Python Agents with rclpy and ROS Controllers
└── Chapter 3: Humanoid Modeling with URDF

Module 2: The Digital Twin (Gazebo & Unity)  <-- CURRENT MODULE
├── Chapter 1: Physics Simulation in Gazebo (gravity, collisions)
├── Chapter 2: Sensors Simulation: LiDAR, Depth Cameras, IMUs
└── Chapter 3: Human-Robot Interaction in Unity

Module 3: Advanced Platforms (NVIDIA Isaac)
├── Chapter 1: NVIDIA Isaac Overview
├── Chapter 2: Isaac ROS Integration
└── Chapter 3: AI Acceleration with NVIDIA Hardware

Module 4: Vision-Language-Action Models (VLA)
├── Chapter 1: Introduction to VLA Models
├── Chapter 2: Implementing VLA for Robotics
└── Chapter 3: Real-World Applications
```

## Development Workflow

### Adding New Content

1. Create a new Markdown file in the appropriate module directory
2. Add it to the sidebar configuration in `sidebars.js`
3. Ensure all content references official documentation sources

### Testing the RAG Functionality

1. Add new content to the book
2. Rebuild the vector database: `python -m services.embedding_service --rebuild`
3. Test the chatbot with questions about the new content

### Validating Content Accuracy

1. Cross-reference all technical claims with official documentation
2. Test all simulation examples in appropriate environments
3. Verify that the RAG system can properly retrieve your content

## Next Steps

1. Complete Module 2: The Digital Twin (Gazebo & Unity)
2. Try the simulation exercises at the end of each chapter
3. Use the chatbot to clarify any concepts you find challenging
4. Move on to Module 3: Advanced Platforms (NVIDIA Isaac)

## Troubleshooting

**Issue**: Chatbot returns "Cannot answer - no relevant book content found"
**Solution**: The question may be outside the scope of the book content. Try rephrasing or refer directly to the book content.

**Issue**: Simulation examples don't work in browser
**Solution**: Some simulation features may require local Gazebo or Unity installations. Check the prerequisites for specific components.

**Issue**: Slow page load times
**Solution**: Clear browser cache and restart the Docusaurus development server.

## Getting Help

- Use the chatbot for content-related questions
- Check the FAQ section in the book
- Report issues in the GitHub repository