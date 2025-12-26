import os
from typing import Optional
from pydantic import BaseModel, Field
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


class Settings(BaseModel):
    """Application settings loaded from environment variables."""

    # API Settings
    api_host: str = Field(default="0.0.0.0", description="Host for the API server")
    api_port: int = Field(default=8000, description="Port for the API server")
    api_debug: bool = Field(default=False, description="Enable debug mode")

    # OpenAI Settings
    openai_api_key: str = Field(..., description="OpenAI API key")
    openai_model: str = Field(default="gpt-4o-mini", description="OpenAI model to use")

    # Qdrant Settings
    qdrant_host: str = Field(default="localhost", description="Qdrant host")
    qdrant_port: int = Field(default=6333, description="Qdrant port")
    qdrant_collection_name: str = Field(default="book_content_chunks", description="Qdrant collection name")

    # Database Settings
    database_url: str = Field(default="sqlite:///./book.db", description="Database URL")

    # Application Settings
    app_name: str = Field(default="AI Robotics Book API", description="Name of the application")
    version: str = Field(default="1.0.0", description="Version of the application")

    class Config:
        # This allows the model to be populated from environment variables
        env_file = ".env"
        case_sensitive = True


def get_settings() -> Settings:
    """Get application settings, loading from environment variables."""
    # Validate required environment variables
    required_vars = ["OPENAI_API_KEY"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

    # Create settings instance
    settings = Settings(
        openai_api_key=os.getenv("OPENAI_API_KEY", ""),
        openai_model=os.getenv("OPENAI_MODEL", "gpt-4o-mini"),
        qdrant_host=os.getenv("QDRANT_HOST", "localhost"),
        qdrant_port=int(os.getenv("QDRANT_PORT", "6333")),
        database_url=os.getenv("DATABASE_URL", "sqlite:///./book.db"),
        api_host=os.getenv("API_HOST", "0.0.0.0"),
        api_port=int(os.getenv("API_PORT", "8000")),
        api_debug=os.getenv("API_DEBUG", "false").lower() == "true",
        app_name=os.getenv("APP_NAME", "AI Robotics Book API"),
        version=os.getenv("VERSION", "1.0.0"),
        qdrant_collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks")
    )

    return settings


# Singleton settings instance
settings = get_settings()