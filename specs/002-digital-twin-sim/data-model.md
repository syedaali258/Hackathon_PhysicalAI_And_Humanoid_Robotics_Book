# Data Model: AI/Spec-Driven Book using Docusaurus - Module 2: Digital Twin

**Feature**: Module 2: The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-21
**Modeler**: Claude Code

## Overview

This document defines the data models for the AI-native book on Physical AI & Humanoid Robotics with embedded RAG chatbot, specifically for Module 2: The Digital Twin (Gazebo & Unity). The models support content management, simulation visualization, user interactions, and RAG functionality while maintaining zero hallucination constraints.

## Core Entities

### 1. BookContent

**Description**: Represents the content of the book including modules, chapters, and sections

**Fields**:
- `id` (string): Unique identifier for the content piece
- `type` (enum): Content type (module, chapter, section, example, exercise, simulation)
- `title` (string): Title of the content piece
- `slug` (string): URL-friendly identifier
- `content` (string): Main content in Markdown/MDX format
- `metadata` (object): Additional content metadata
  - `author` (string): Content author
  - `lastUpdated` (date): Last modification date
  - `version` (string): Content version
  - `tags` (array): Content tags for categorization
  - `prerequisites` (array): Prerequisite content IDs
  - `simulationType` (string): Type of simulation (gazebo, unity, mixed) - specific to Module 2
- `hierarchy` (object): Position in the book structure
  - `moduleNumber` (integer): Module sequence number
  - `chapterNumber` (integer): Chapter sequence number within module
  - `sectionNumber` (integer): Section sequence number within chapter
- `references` (array): Official documentation references for validation

**Relationships**:
- Parent: One parent content piece (except modules)
- Children: Multiple child content pieces
- Prerequisites: Multiple prerequisite content pieces

**Validation Rules**:
- Title must be 3-100 characters
- Slug must be URL-friendly (alphanumeric, hyphens, underscores)
- Content must be in valid Markdown/MDX format
- Prerequisites must exist in the system
- SimulationType must be one of: gazebo, unity, mixed, none

### 2. UserSession

**Description**: Represents a user's interaction session with the book and chatbot

**Fields**:
- `id` (string): Unique session identifier
- `userId` (string): User identifier (anonymous or authenticated)
- `startTime` (datetime): Session start time
- `endTime` (datetime): Session end time (null if active)
- `currentLocation` (object): Current position in the book
  - `moduleId` (string): Current module ID
  - `chapterId` (string): Current chapter ID
  - `sectionId` (string): Current section ID
- `progress` (array): User progress tracking
  - `contentId` (string): Content piece ID
  - `completionStatus` (enum): not_started, in_progress, completed
  - `timeSpent` (integer): Time spent in seconds
  - `lastAccessed` (datetime): Last access time
- `simulationProgress` (array): Simulation-specific progress - Module 2 specific
  - `simulationId` (string): ID of the simulation component
  - `completionStatus` (enum): not_started, in_progress, completed
  - `attempts` (integer): Number of attempts at simulation exercises

**Relationships**:
- User: Associated user (if authenticated)
- CurrentContent: Currently viewed content piece
- Progress: Multiple content progress records
- SimulationProgress: Multiple simulation progress records

**Validation Rules**:
- Session must have a start time
- End time must be after start time (if set)
- Current location must reference valid content
- Simulation progress must reference valid simulation components

### 3. ChatMessage

**Description**: Represents a message in the RAG-enabled chatbot conversation

**Fields**:
- `id` (string): Unique message identifier
- `sessionId` (string): Associated user session ID
- `timestamp` (datetime): Message timestamp
- `sender` (enum): sender type (user, assistant)
- `content` (string): Message content
- `sourceChunks` (array): Source content chunks used for response
  - `chunkId` (string): Content chunk identifier
  - `text` (string): Chunk text content
  - `relevanceScore` (number): Relevance score (0-1)
  - `contentId` (string): Original content piece ID
- `confidence` (number): Confidence score of the response (0-1)
- `followUpSuggestions` (array): Suggested follow-up questions
- `simulationContext` (object): Context related to simulation content - Module 2 specific
  - `currentSimulation` (string): Currently referenced simulation
  - `environmentType` (string): Gazebo or Unity environment
  - `physicsParameters` (object): Relevant physics parameters

**Relationships**:
- Session: Associated user session
- SourceContent: Multiple source content pieces used in response

**Validation Rules**:
- Content must not exceed 10,000 characters
- Confidence score must be between 0 and 1
- Source chunks must reference valid content pieces
- Confidence must be high (>0.7) for responses
- Simulation context must reference valid simulation components

### 4. ContentChunk

**Description**: Represents a chunk of content used for RAG retrieval

**Fields**:
- `id` (string): Unique chunk identifier
- `contentId` (string): Original content piece ID
- `text` (string): Chunk text content
- `embedding` (array): Vector embedding for similarity search
- `chunkIndex` (integer): Position of chunk in original content
- `metadata` (object): Additional metadata
  - `contentType` (string): Type of content in chunk
  - `keywords` (array): Keywords extracted from chunk
  - `summary` (string): Brief summary of chunk content
  - `simulationRelevance` (boolean): Whether chunk relates to simulation content - Module 2 specific
  - `simulationType` (string): Gazebo, Unity, or mixed simulation type
  - `physicsConcepts` (array): Physics concepts covered in chunk

**Relationships**:
- OriginalContent: Source content piece
- ChatMessages: Multiple chat messages that reference this chunk

**Validation Rules**:
- Text must be between 50 and 1000 characters
- Embedding must have consistent dimensions
- Chunk index must be non-negative
- Simulation relevance must be boolean
- Simulation type must be one of: gazebo, unity, mixed, none

### 5. SimulationComponent

**Description**: Represents interactive simulation components specific to Module 2 - Digital Twin

**Fields**:
- `id` (string): Unique identifier for the simulation component
- `name` (string): Name of the simulation component
- `type` (enum): Type of simulation (gazebo, unity, visualization, interaction)
- `description` (string): Description of the simulation component
- `contentId` (string): Associated content piece ID
- `parameters` (object): Configuration parameters for the simulation
  - `physicsEngine` (string): Physics engine used (for Gazebo components)
  - `visualizationSettings` (object): Settings for Unity components
  - `sensorTypes` (array): Types of sensors simulated (LiDAR, camera, IMU, etc.)
- `metadata` (object): Additional metadata
  - `difficultyLevel` (enum): beginner, intermediate, advanced
  - `estimatedTime` (integer): Estimated time to complete in minutes
  - `learningObjectives` (array): Educational objectives of the component
- `createdAt` (datetime): Creation timestamp
- `updatedAt` (datetime): Last update timestamp

**Relationships**:
- Content: Associated book content piece
- UserSessions: Multiple user sessions that interacted with this component

**Validation Rules**:
- Name must be 3-100 characters
- Type must be one of: gazebo, unity, visualization, interaction
- Difficulty level must be one of: beginner, intermediate, advanced
- Estimated time must be positive
- Parameters must be valid for the specified simulation type

### 6. ModuleProgress

**Description**: Tracks user progress through specific modules

**Fields**:
- `id` (string): Unique progress record identifier
- `userId` (string): User identifier
- `moduleId` (string): Module being tracked
- `completionPercentage` (number): Percentage of module completed (0-100)
- `timeSpent` (integer): Total time spent on module in seconds
- `lastAccessed` (datetime): Last time module was accessed
- `completedChapters` (array): IDs of completed chapters
- `simulationActivitiesCompleted` (integer): Count of completed simulation activities - Module 2 specific
- `physicsConceptsMastered` (array): Physics concepts the user has mastered
- `quizScores` (array): Scores for module quizzes
  - `quizId` (string): Quiz identifier
  - `score` (number): Score achieved (0-100)
  - `attempts` (integer): Number of attempts

**Relationships**:
- User: Associated user
- Module: Tracked module
- CompletedChapters: Multiple completed chapters

**Validation Rules**:
- Completion percentage must be between 0 and 100
- Time spent must be non-negative
- Quiz scores must be between 0 and 100
- Simulation activities completed must be non-negative
- Physics concepts mastered must reference valid concepts

## State Transitions

### UserSession States
1. **Active**: Session is currently in progress
2. **Paused**: Session temporarily inactive
3. **Completed**: Session has ended

Transition Rules:
- Active → Paused: User navigates away for >5 minutes
- Active → Completed: User explicitly ends session or is inactive for >30 minutes
- Paused → Active: User returns to session within timeout period

### Content Validation States
1. **Draft**: Content is being created
2. **UnderReview**: Content is being validated
3. **Validated**: Content has passed accuracy checks
4. **Published**: Content is available to users

Transition Rules:
- Draft → UnderReview: Content is submitted for validation
- UnderReview → Validated: Content passes accuracy checks against official sources
- UnderReview → Draft: Content requires revisions
- Validated → Published: Content is approved for publication
- Published → UnderReview: Content is updated and needs re-validation

## Relationships Summary

```
BookContent (1) ← → (0..n) BookContent (Parent-Child)
BookContent (1) ← → (0..n) ContentChunk (Content-Chunks)
BookContent (1) ← → (0..n) SimulationComponent (Content-Simulation)
UserSession (1) ← → (0..n) ChatMessage (Session-Messages)
UserSession (1) ← → (0..n) ModuleProgress (User-Progress)
SimulationComponent (1) ← → (0..n) UserSession (Simulation-Session)
ChatMessage (0..n) ← → (0..n) ContentChunk (Message-Sources)
ModuleProgress (1) ← → (0..n) SimulationComponent (Progress-Simulation)
```

## Indexes for Performance

1. **BookContent**: Index on (type, moduleNumber, chapterNumber, sectionNumber, simulationType) for navigation and simulation filtering
2. **ChatMessage**: Index on (sessionId, timestamp) for conversation retrieval
3. **ContentChunk**: Index on (contentId, chunkIndex, simulationRelevance) for content reconstruction and simulation-specific retrieval
4. **UserSession**: Index on (userId, startTime) for session tracking
5. **ModuleProgress**: Index on (userId, moduleId) for progress tracking
6. **SimulationComponent**: Index on (type, difficultyLevel, contentId) for simulation component retrieval