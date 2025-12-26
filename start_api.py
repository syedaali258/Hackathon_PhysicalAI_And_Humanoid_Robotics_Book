import os
import sys

# Add the project root and api directory to the Python path
project_root = os.path.dirname(__file__)
api_dir = os.path.join(project_root, 'api')

# Insert both directories to the path
sys.path.insert(0, project_root)
sys.path.insert(0, api_dir)

# Now run the uvicorn server
if __name__ == "__main__":
    import uvicorn

    # Import the app after setting up the path
    os.chdir(api_dir)  # Change working directory for file operations
    import main
    uvicorn.run(main.app, host="127.0.0.1", port=8000, reload=True)