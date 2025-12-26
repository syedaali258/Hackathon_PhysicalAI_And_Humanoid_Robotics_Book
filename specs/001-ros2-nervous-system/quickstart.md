# Quickstart Guide: AI/Spec-Driven Book using Docusaurus

**Feature**: Module 1: The Robotic Nervous System (ROS 2)
**Date**: 2025-12-20

## Overview

This guide provides a quick setup and initial experience for the AI-native book on Physical AI & Humanoid Robotics. Follow these steps to get started with Module 1: The Robotic Nervous System (ROS 2).

## Prerequisites

- Node.js 18+ installed
- Python 3.8+ installed
- Basic understanding of command line tools
- Web browser (Chrome, Firefox, or Safari recommended)

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
2. You'll see the book homepage with Module 1: The Robotic Nervous System
3. The RAG chatbot will be available on every page

## First Experience: Module 1

### Navigate to Module 1

1. Click on "Module 1: The Robotic Nervous System (ROS 2)" in the sidebar
2. Start with Chapter 1: "ROS 2 Fundamentals: Nodes, Topics, and Services"

### Try the Interactive Features

1. **Read the content**: Each chapter includes conceptual explanations with diagrams
2. **Use the chatbot**: Ask questions about the content using the chat widget
   - Example: "Explain the difference between ROS 2 topics and services"
   - The chatbot will respond based only on book content with source citations
3. **Try code examples**: Each chapter includes practical Python examples with rclpy

### Sample Interaction with RAG Chatbot

```
You: "What are the main differences between ROS 2 nodes, topics, and services?"
Bot: "In ROS 2:

- **Nodes** are individual processes that perform computation. They are the basic building blocks of a ROS 2 system.
- **Topics** provide a publish-subscribe communication pattern for continuous data streams (like sensor data).
- **Services** provide request-response communication for specific tasks that require a reply.

Sources: [Module 1, Chapter 1: ROS 2 Fundamentals], [Module 1, Chapter 2: Python Agents]

Follow-up questions:
- "Can you give an example of when to use topics vs services?"
- "How do ROS 2 nodes discover each other?"
```

## Content Structure

The book is organized as follows:

```
Module 1: The Robotic Nervous System (ROS 2)
├── Chapter 1: ROS 2 Fundamentals: Nodes, Topics, and Services
├── Chapter 2: Python Agents with rclpy and ROS Controllers
└── Chapter 3: Humanoid Modeling with URDF

Module 2: Simulation Environments (Gazebo/Unity)
├── Chapter 1: Gazebo Basics for Robotics
├── Chapter 2: Unity Robotics Hub Integration
└── Chapter 3: Simulation-Based Learning

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
2. Test all code examples in simulation environments
3. Verify that the RAG system can properly retrieve your content

## Next Steps

1. Complete Module 1: The Robotic Nervous System
2. Try the exercises at the end of each chapter
3. Use the chatbot to clarify any concepts you find challenging
4. Move on to Module 2: Simulation Environments

## Troubleshooting

**Issue**: Chatbot returns "Cannot answer - no relevant book content found"
**Solution**: The question may be outside the scope of the book content. Try rephrasing or refer directly to the book content.

**Issue**: Examples don't work in simulation
**Solution**: Ensure you have the correct ROS 2 distribution installed and properly sourced.

**Issue**: Slow page load times
**Solution**: Clear browser cache and restart the Docusaurus development server.

## Getting Help

- Use the chatbot for content-related questions
- Check the FAQ section in the book
- Report issues in the GitHub repository