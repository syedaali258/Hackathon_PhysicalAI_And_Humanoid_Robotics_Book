from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from api.routers import chat_router, content_router, vla_router
from api.utils.logging import logger
from api.utils.config import settings
from api.utils.security import SecurityHeaders, rate_limiter
import uvicorn
import time


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for application startup and shutdown.
    """
    # Startup
    logger.info("Starting up AI Robotics Book API")
    yield
    # Shutdown
    logger.info("Shutting down AI Robotics Book API")


# Create FastAPI app instance
app = FastAPI(
    title=settings.app_name,
    version=settings.version,
    debug=settings.api_debug,
    description="API for the AI-native book on Physical AI & Humanoid Robotics with embedded RAG chatbot",
    lifespan=lifespan,
    # Add security-related configurations
    root_path=settings.api_root_path if hasattr(settings, 'api_root_path') else "",
    # Prevents information disclosure
    docs_url="/docs" if settings.api_debug else None,
    redoc_url="/redoc" if settings.api_debug else None,
    openapi_url="/openapi.json" if settings.api_debug else None
)

# Add security and performance middleware
app.add_middleware(
    GZipMiddleware,
    minimum_size=1000,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Security: Add extra headers to prevent common attacks
    allow_origin_regex=r"https://.*\.github\.io"  # Allow GitHub Pages
)


# Add security middleware
@app.middleware("http")
async def security_headers_middleware(request: Request, call_next):
    """Add security headers to all responses."""
    response = await call_next(request)
    response = SecurityHeaders.add_security_headers(request, response)
    return response


@app.middleware("http")
async def rate_limit_middleware(request: Request, call_next):
    """Apply rate limiting to requests."""
    # Skip rate limiting for health checks
    if request.url.path in ["/", "/health"]:
        response = await call_next(request)
        return response

    # Use client IP for rate limiting
    client_ip = request.client.host if request.client else "unknown"

    if not rate_limiter.is_allowed(client_ip):
        from fastapi.responses import JSONResponse
        return JSONResponse(
            status_code=429,
            content={"detail": "Rate limit exceeded. Please try again later."}
        )

    response = await call_next(request)
    return response


# Add middleware for logging API requests
@app.middleware("http")
async def log_requests(request: Request, call_next):
    start_time = time.time()

    # Skip logging for health checks to reduce noise
    if request.url.path not in ["/", "/health"]:
        logger.info(
            f"API request started",
            extra={
                "method": request.method,
                "path": request.url.path,
                "client": request.client.host if request.client else None
            }
        )

    response = await call_next(request)

    process_time = time.time() - start_time
    if request.url.path not in ["/", "/health"]:
        logger.info(
            f"API request completed",
            extra={
                "method": request.method,
                "path": request.url.path,
                "status_code": response.status_code,
                "process_time": f"{process_time:.4f}s"
            }
        )

    return response


# Include routers
app.include_router(chat_router)
app.include_router(content_router)
app.include_router(vla_router)


@app.get("/")
async def root():
    """Root endpoint for health check."""
    logger.info("Health check endpoint accessed")
    return {
        "message": "AI Robotics Book API",
        "version": settings.version,
        "status": "running"
    }


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    logger.info("Detailed health check endpoint accessed")
    return {
        "status": "healthy",
        "version": settings.version,
        "timestamp": __import__('datetime').datetime.now().isoformat()
    }


# If running this file directly, start the server
if __name__ == "__main__":
    logger.info(f"Starting {settings.app_name} on {settings.api_host}:{settings.api_port}")
    uvicorn.run(
        "main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=settings.api_debug,
    )