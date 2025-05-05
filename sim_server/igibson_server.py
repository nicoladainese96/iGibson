
import base64
from io import BytesIO
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Dict, Any, Optional, List
import traceback
from igibson.action_primitives.fetch_robot_semantic_actions_env import FetchRobotSemanticActionEnv


# Create FastAPI app
app = FastAPI(title="iGibson Robot Action API")

# Define request and response models
class ActionParams(BaseModel):
    obj_name: str

class ActionRequest(BaseModel):
    action: str
    params: List[str] #ActionParams

class ActionResponse(BaseModel):
    success: bool
    image: str  # Base64 encoded image
    symbolic_state: Optional[Dict[str, Any]] = {}  # Make it optional with a default empty dict

# Define request model for reset endpoint
class ResetRequest(BaseModel):
    task: str
    scene_id: str

# Initialize environment globally with default values 
default_task = "cleaning_out_drawers"
default_scene_id = "Benevolence_1_int"
env = FetchRobotSemanticActionEnv(default_task, default_scene_id, verbose=False)

@app.post("/execute_action", response_model=ActionResponse)
async def execute_action(action_request: ActionRequest):
    print("-----------------------------------------")
    print(f"Got new action request: {action_request}")
    try:
        # Format the action in the expected structure
        action = {
            'action': action_request.action,
            'params': action_request.params
        }
        
        # Apply the action
        success, image, symbolic_state = env.apply_action(action)
        print(f"Action {action_request.action} executed. Success: {success}")
        
        # Convert image to base64 string
        buffered = BytesIO()
        image.save(buffered, format="PNG")
        img_str = base64.b64encode(buffered.getvalue()).decode()
        
        # Return the response with a default empty dict if symbolic_state is None
        return ActionResponse(
            success=success,
            image=img_str,
            symbolic_state=symbolic_state if symbolic_state is not None else {}
        )
    except Exception as e:
        print(f"Error executing action: {action_request.action} with params: {action_request.params}")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Error executing action: {str(e)}")

@app.post("/reset")
async def reset_environment(reset_request: ResetRequest):
    print("-----------------------------------------")
    print(f"Resetting environment with task: {reset_request.task}, scene_id: {reset_request.scene_id}")
    try:
        global env
        # Use task and scene_id from the request body
        env = FetchRobotSemanticActionEnv(reset_request.task, reset_request.scene_id, verbose=False)

        return {
            "success": True
        }
    except Exception as e:
        print("Error during environment reset.")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Error resetting environment: {str(e)}")

@app.get("/get_state")
async def get_environment_state():
    print("-----------------------------------------")
    print("Getting environment state")
    
    try:
        # Get the current state and image
        image, symbolic_state = env.get_state_and_image()
        
        # Convert image to base64 string
        buffered = BytesIO()
        image.save(buffered, format="PNG")
        img_str = base64.b64encode(buffered.getvalue()).decode()
        
        return {
            "success": True,
            "image": img_str,
            "symbolic_state": symbolic_state
        }
    except Exception as e:
        print("Error getting environment state.")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Error getting environment state: {str(e)}")

