# Quickstart Guide: AI/Spec-Driven Book using Docusaurus

**Feature**: AI-native book on Physical AI & Humanoid Robotics with embedded RAG chatbot
**Branch**: 004-vla-module
**Date**: 2025-12-24

## Overview

This guide provides a quick start for developers and educators to set up, run, and contribute to the AI-native book on Physical AI & Humanoid Robotics with embedded RAG chatbot.

## Prerequisites

### System Requirements
- **Node.js**: 18+ (for Docusaurus frontend)
- **Python**: 3.8+ (for FastAPI backend)
- **Git**: Version control
- **OpenAI API Key**: For RAG functionality (optional for basic functionality)

### Recommended Environment
- **OS**: Linux, macOS, or Windows with WSL2
- **Memory**: 8GB+ RAM recommended
- **Disk**: 2GB+ free space

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-robotics-book
```

### 2. Set up Backend (FastAPI)
```bash
# Navigate to backend directory
cd api

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env to add your OpenAI API key (optional)
```

### 3. Set up Frontend (Docusaurus)
```bash
# Navigate to book directory
cd book

# Install dependencies
npm install
```

### 4. Set up Vector Database (Qdrant)
```bash
# Option 1: Docker (recommended for development)
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant

# Option 2: Local installation (follow Qdrant documentation)
```

## Running the Application

### 1. Start the Backend
```bash
# From the api directory
cd api
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn main:app --reload --port 8000
```

### 2. Start the Frontend
```bash
# From the book directory
cd book
npm start
```

### 3. Access the Application
- **Frontend**: http://localhost:3000
- **Backend API**: http://localhost:8000
- **Backend API Docs**: http://localhost:8000/docs

## Key Features

### 1. Module Navigation
- Navigate through 4 main modules: ROS 2, Digital Twin, AI Brain, VLA
- Each module contains 3-5 chapters with progressive difficulty
- Interactive content and code examples

### 2. RAG Chatbot
- Ask questions about book content
- Zero hallucination responses based only on book content
- Source citations for all responses

### 3. VLA (Vision-Language-Action) Module
- Voice command processing
- Language-based planning with LLMs
- Action execution simulation
- Complete autonomous humanoid demonstrations

## Development Workflow

### Adding New Content
```bash
# 1. Create new markdown file in appropriate module directory
book/docs/module-4-vla/new-content.md

# 2. Add to sidebar navigation
book/sidebars.ts

# 3. Ensure content follows APA-style citations
# 4. Validate against official documentation sources
```

### Running Tests
```bash
# Backend tests
cd api
python -m pytest tests/ -v

# Frontend tests
cd book
npm test
```

### Content Validation
```bash
# Ensure all content is technically accurate
# Verify against official documentation
# Check for zero hallucination compliance
```

## API Endpoints

### Chat Endpoints
- `POST /chat/start` - Start new chat session
- `POST /chat/{sessionId}/message` - Send message to chatbot
- `GET /chat/{sessionId}/history` - Get chat history
- `POST /chat/{sessionId}/end` - End chat session

### Content Endpoints
- `POST /content/search` - Semantic search across book content

### VLA Endpoints
- `POST /vla/process` - Process VLA requests
- `POST /vla/plan` - Generate action plans
- `POST /vla/execute-plan` - Execute action plans

## Configuration

### Environment Variables
```bash
# api/.env
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-4o-mini
QDRANT_HOST=localhost
QDRANT_PORT=6333
API_HOST=0.0.0.0
API_PORT=8000
API_DEBUG=true
```

### Docusaurus Configuration
- Located in `book/docusaurus.config.ts`
- Configure site metadata, navigation, and plugins

## Deployment

### GitHub Pages Deployment
```bash
# Build the frontend
cd book
npm run build

# The output will be in the build/ directory
# Configure GitHub Pages to serve from the build/ directory
```

### Backend Deployment
- Deploy FastAPI backend to cloud provider (AWS, GCP, Azure, etc.)
- Ensure Qdrant database is accessible
- Configure environment variables for production

## Troubleshooting

### Common Issues
1. **Port conflicts**: Ensure ports 3000 and 8000 are available
2. **API key issues**: Verify OpenAI API key is correctly set
3. **Database connectivity**: Check Qdrant is running and accessible
4. **Content not searchable**: Re-index content after updates

### Development Mode
- Use `uvicorn main:app --reload` for auto-restart on code changes
- Frontend automatically reloads on file changes

## Contributing

### Content Guidelines
1. Follow APA-style citations
2. Verify all technical claims against official documentation
3. Maintain zero hallucination constraint for chatbot responses
4. Include practical examples and code snippets
5. Ensure content is accessible to robotics learners

### Code Standards
1. Write comprehensive tests for new functionality
2. Follow existing code style and patterns
3. Document API endpoints with examples
4. Maintain backward compatibility when possible

## Next Steps

1. **Explore the modules**: Start with Module 1 (ROS 2) and progress through to Module 4 (VLA)
2. **Try the chatbot**: Ask questions about the book content to experience the RAG functionality
3. **Experiment with VLA**: Test voice commands and see the action planning process
4. **Contribute content**: Add new examples, exercises, or chapters to enhance the educational value

This quickstart guide should get you up and running with the AI-native robotics book platform in just a few minutes!