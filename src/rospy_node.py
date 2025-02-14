#!/usr/bin/env python
"""
A ROS node that loads a neural network (e.g., GatCNMP), computes a route,
and publishes it as a nav_msgs/Path message. The idea is that your C++ global
planner can subscribe to this topic to get the “true” plan instead of the
default straight-line plan.
"""

import rospy
import torch
import torch.nn.functional as F
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

# Import your network definition. Make sure the parent directory is in PYTHONPATH.
from gat_cnmp import GatCNMP

# (Optional) For handling PyG Data and Batch objects.
from torch_geometric.data import Data, Batch

def get_free_gpu():
    """
    Utility to pick the GPU with lowest utilization.
    """
    gpu_util = []
    for i in range(torch.cuda.device_count()):
        torch.cuda.set_device(i)  # Switch GPU
        gpu_util.append((i, torch.cuda.utilization()))
    gpu_util.sort(key=lambda x: x[1])
    return gpu_util[0][0]

def load_model():
    """
    Loads the neural network model and its weights.
    """
    if torch.cuda.is_available():
        available_gpu = get_free_gpu()
        device = torch.device(f"cuda:{available_gpu}")
    else:
        device = torch.device("cpu")
    rospy.loginfo("Using device: %s", device)

    # Initialize your model (adjust parameters as needed)
    model = GatCNMP(gat_out_channels=32).to(device)
    if torch.__version__ >= "2.0":
        model = torch.compile(model)
        
    # Load the model weights (adjust the path to your saved model)
    timestamp = '1738319077'
    model_path = f'output/simulation/{timestamp}/saved_model/on_sim.pt'
    model.load_state_dict(torch.load(model_path))
    model.eval()
    rospy.loginfo("Neural network model loaded.")
    return model, device

def get_nn_plan(model, device):
    """
    Queries the neural network to compute a plan.
    
    For this demonstration, dummy input data is used.
    In your final version, you would use the actual environment state,
    start/goal positions, and any other required inputs.
    
    Returns:
        x_vals (np.ndarray): x coordinates of the route.
        y_vals (np.ndarray): y coordinates (predicted by the model) of the route.
    """
    # --- Dummy input creation ---
    # For demonstration we assume a plan with t_steps time points.
    batch_size = 1
    t_steps = 50
    n_max = 10  # e.g., number of observed nodes
    dx, dy = 1, 1

    # Create a dummy observation tensor (e.g., representing observed positions)
    val_obs = torch.zeros((batch_size, n_max, dx + dy), dtype=torch.float32, device=device)
    # (Fill in with real observation data as needed)

    # Create a dummy target tensor for x-coordinates (e.g., as a time/index series)
    val_tar_x = torch.linspace(0, 10, t_steps).unsqueeze(1).unsqueeze(0).to(device)  # shape: (1, t_steps, 1)

    # Create dummy graphs for each observed node.
    graphs = []
    for i in range(n_max):
        # For demonstration, create a trivial graph with one node and no edges.
        node_feature = torch.tensor([[0.0]], dtype=torch.float32)
        edge_index = torch.empty((2, 0), dtype=torch.long)
        graphs.append(Data(x=node_feature, edge_index=edge_index))
    graph_batch = Batch.from_data_list(graphs).to(device)

    # --- Run the model ---
    with torch.no_grad():
        out = model(graph_batch, val_obs, val_tar_x)
    
    # Assume the model outputs a tensor of shape (batch_size, t_steps, 2)
    # where index 0 is the predicted "mean" (e.g., y-coordinate) and index 1 is a logit for std.
    # (You might need to adjust this depending on your model's actual output.)
    x_vals = val_tar_x[0, :, 0].cpu().numpy()  # x coordinates (e.g., time or spatial positions)
    # Use the predicted mean as the y-coordinate for the route.
    y_vals = out[0, :, 0].cpu().numpy()
    
    return x_vals, y_vals

def main():
    rospy.init_node('nn_global_planner_node', anonymous=False)
    plan_pub = rospy.Publisher('nn_global_plan', Path, queue_size=1)
    
    # Load the neural network model
    model, device = load_model()
    
    rate = rospy.Rate(0.5)  # Plan and publish at 0.5 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Computing plan using neural network...")
        x_vals, y_vals = get_nn_plan(model, device)
        
        # Build the Path message from the predicted route
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"  # Ensure this matches your global frame
        
        num_points = len(x_vals)
        for i in range(num_points):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x_vals[i]
            pose.pose.position.y = y_vals[i]
            pose.pose.position.z = 0.0
            # Use a default orientation (no rotation)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        plan_pub.publish(path_msg)
        rospy.loginfo("Published NN global plan with %d points.", num_points)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
