import requests
import json
import base64
from PIL import Image
import io
import time
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Test API endpoints.")
parser.add_argument('--base_url', type=str, default="http://gpu36:8000", help='Base URL for the API')
args = parser.parse_args()

# Global base URL
base_url = args.base_url

# Test the reset endpoint
def test_reset(task, scene_id):
    payload = {
        "task": task,
        "scene_id": scene_id
    }
    start_time = time.time()
    
    response = requests.post(f"{base_url}/reset", json=payload)  
    end_time = time.time()
    execution_time = end_time - start_time
    
    if response.status_code == 200:
        data = response.json()
        print(f"\nReset successful: {data['success']}")
        print(f"Execution time: {execution_time:.4f} seconds")
    else:
        print(f"Reset failed with status code {response.status_code}")
        print(response.text)

def test_get_state():
    start_time = time.time()
    response = requests.get(f"{base_url}/get_state")
    end_time = time.time()
    execution_time = end_time - start_time
    
    if response.status_code == 200:
        data = response.json()
        print(f"\nGet state successful: {data['success']}")
        print(f"Execution time: {execution_time:.4f} seconds")
        img_data = base64.b64decode(data['image'])
        img = Image.open(io.BytesIO(img_data))
        img.save("state_image.png")
        print("Image saved as state_image.png")
        # Print symbolic state
        print("\nSymbolic state:")
        print(json.dumps(data['symbolic_state'], indent=2))
    else:
        print(f"Get state failed with status code {response.status_code}")
        print(response.text)

# Test the execute_action endpoint
def test_execute_action(action, obj_name):
    payload = {
        "action": action,
        "params": [obj_name]
    }
    
    start_time = time.time()
    response = requests.post(f"{base_url}/execute_action", json=payload)
    end_time = time.time()
    execution_time = end_time - start_time
    
    if response.status_code == 200:
        data = response.json()
        print(f"\nAction execution successful: {data['success']}")
        print(f"Execution time: {execution_time:.4f} seconds")
        
        # Decode and save the image
        img_data = base64.b64decode(data['image'])
        img = Image.open(io.BytesIO(img_data))
        img.save(f"{action}_{obj_name}.png")
        print(f"Image saved as {action}_{obj_name}.png")
        
        # Print symbolic state
        print("Symbolic state:")
        print(json.dumps(data['symbolic_state'], indent=2))
    else:
        print(f"Action execution failed with status code {response.status_code}")
        print(response.text)
    return execution_time

if __name__ == "__main__":
    # Test reset first
    test_reset("cleaning_out_drawers","Benevolence_1_int")
    # Then test get_state
    test_get_state()
    # Test actions
    test_execute_action("navigate-to", "cabinet_1")
    test_execute_action("open-container", "cabinet_1")
    test_execute_action("grasp", "bowl_1")

