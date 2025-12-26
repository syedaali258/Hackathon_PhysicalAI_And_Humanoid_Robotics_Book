# AI/Robotics Book - Module 1: The Robotic Nervous System (ROS 2)

This project implements an AI-native book on Physical AI & Humanoid Robotics with embedded RAG chatbot, focusing on Module 1: The Robotic Nervous System (ROS 2).

## Table of Contents
- [Overview](#overview)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Application](#running-the-application)
- [Development](#development)
- [Features](#features)

## Overview

This AI-native book provides interactive learning materials for ROS 2 concepts including:
- ROS 2 fundamentals (nodes, topics, services)
- Python agents with rclpy and ROS controllers
- Humanoid modeling with URDF

The system features a RAG (Retrieval-Augmented Generation) chatbot that answers questions based only on book content, ensuring zero hallucination.

## Architecture

The application consists of two main components:
1. **Frontend**: Docusaurus-based book interface running on port 3000
2. **Backend**: FastAPI API server with RAG functionality running on port 8000

## Prerequisites

- Node.js 18+ with npm
- Python 3.8+
- An OpenAI API key (for RAG functionality)

## Installation

### 1. Clone the repository
```bash
git clone <repository-url>
cd ai-robotics-book
```

### 2. Install frontend dependencies
```bash
cd book
npm install
```

### 3. Install backend dependencies
```bash
cd ../api
pip install -r requirements.txt
```

### 4. Set up environment variables
Create a `.env` file in the `api` directory with your OpenAI API key:

```bash
# In the api directory
OPENAI_API_KEY=your_openai_api_key_here
```

See `.env.example` for a complete list of available configuration options.

## Running the Application

The application requires both the frontend and backend to run simultaneously.

### Terminal 1 - Start the Frontend (Docusaurus)
```bash
cd book
npm start
```
The frontend will be available at `http://localhost:3000`

### Terminal 2 - Start the Backend (FastAPI)
```bash
cd api
uvicorn main:app --reload --port 8000
```
The backend API will be available at `http://localhost:8000`

API documentation will be available at `http://localhost:8000/docs` when running in debug mode.

## Development

### Adding New Content
1. Create new Markdown files in the appropriate module directory (e.g., `book/docs/module-1-ros2/`)
2. Add the new content to the sidebar configuration in `book/sidebars.ts`
3. The content will be automatically processed by the RAG system

### Running Tests
Backend tests are located in `api/tests/`:
```bash
cd api
python -m pytest tests/ -v
```

### Environment Configuration
The application can be configured using environment variables in the `.env` file:

| Variable | Default | Description |
|----------|---------|-------------|
| OPENAI_API_KEY | | Your OpenAI API key (required) |
| OPENAI_MODEL | gpt-4o-mini | OpenAI model to use |
| QDRANT_HOST | localhost | Qdrant vector database host |
| QDRANT_PORT | 6333 | Qdrant vector database port |
| API_HOST | 0.0.0.0 | API server host |
| API_PORT | 8000 | API server port |
| API_DEBUG | false | Enable debug mode |

## Features

### Interactive Learning Components
- ROS 2 communication diagram visualization
- Python agent examples with rclpy
- URDF robot model viewer with joint controls
- ROS controller simulation

### RAG Chatbot
- Questions answered based only on book content
- Source citations for all responses
- Context-aware follow-up suggestions
- Zero hallucination guarantee

### Content Search
- Semantic search across all book content
- Relevance scoring
- Filtering capabilities

### Security Features
- Rate limiting (100 requests per hour per IP)
- Input validation and sanitization
- Security headers on all responses
- API key authentication (planned)

## API Endpoints

### Chat Endpoints
- `POST /chat/start` - Start a new chat session
- `POST /chat/{sessionId}/message` - Send a message to the chatbot
- `GET /chat/{sessionId}/history` - Get chat history
- `POST /chat/{sessionId}/end` - End a chat session

### Content Endpoints
- `POST /content/search` - Search book content

## Deployment

The frontend is designed for GitHub Pages deployment:
1. Build the frontend: `npm run build`
2. The output will be in the `build/` directory
3. Configure GitHub Pages to serve from the `build/` directory

## Troubleshooting

### Common Issues
1. **OpenAI API errors**: Verify your API key is correct and has sufficient quota
2. **Backend not connecting to frontend**: Ensure both servers are running on their respective ports
3. **Content not appearing in search**: Content is processed automatically, but may take a moment to become searchable

### Development Mode
For development, use `uvicorn main:app --reload` to automatically restart the server when code changes are detected.

## Next Steps

Future modules will cover:
- Module 2: Simulation Environments (Gazebo/Unity)
- Module 3: Advanced Platforms (NVIDIA Isaac)
- Module 4: Vision-Language-Action Models (VLA)