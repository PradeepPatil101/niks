import open3d as o3d
import os
import time
import natsort  # To sort filenames naturally
import numpy as np

def play_pcd_sequence(pcd_folder, frame_delay=0.1):
    # Get list of all .pcd files in the folder
    pcd_files = [f for f in os.listdir(pcd_folder) if f.endswith('.pcd')]
    
    # Sort files in natural order (Frame 1, Frame 2, ... Frame 100)
    pcd_files = natsort.natsorted(pcd_files)
    
    if not pcd_files:
        print("No .pcd files found in the folder.")
        return

    print(f"Playing {len(pcd_files)} point cloud frames...")

    # Create Open3D Visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add coordinate frame to show axes
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20.0)  # You can adjust the size
    vis.add_geometry(coordinate_frame)

    # Define a 90-degree rotation matrix to orient axes: X forward, Y left, Z up
    # Note: This is a rotation around the Z-axis (yaw) by 90 degrees counter-clockwise
    rotation_matrix = np.array([
        [0, -1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    rotation_matrix_2 = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])

    # Load first frame and apply the rotation
    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[0]))
    pcd.transform(rotation_matrix) 
    pcd.transform(rotation_matrix_2) 
    vis.add_geometry(pcd)

    for pcd_file in pcd_files:
        pcd_path = os.path.join(pcd_folder, pcd_file)
        new_pcd = o3d.io.read_point_cloud(pcd_path)

        if new_pcd.is_empty():
            print(f"Skipping empty PCD: {pcd_file}")
            continue

        print(f"Displaying: {pcd_file}")

        # Apply the same rotation to the new frame
        new_pcd.transform(rotation_matrix)
        new_pcd.transform(rotation_matrix_2) 

        # Apply DBSCAN clustering
        labels = np.array(new_pcd.cluster_dbscan(eps=0.5, min_points=1))  # Adjust eps and min_points as needed
        max_label = labels.max()

        print(f"Point cloud has {max_label + 1} clusters")

        # Color the clusters (optional, for visualization)
        colors = np.random.uniform(0, 1, size=(max_label + 1, 3))
        colored_points = np.zeros((len(new_pcd.points), 3))
        for i in range(len(new_pcd.points)):
            if labels[i] >= 0:
                colored_points[i] = colors[labels[i]]
        new_pcd.colors = o3d.utility.Vector3dVector(colored_points)

        # Update the existing point cloud instead of re-adding
        pcd.points = new_pcd.points
        pcd.colors = new_pcd.colors  # Preserve colors
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        time.sleep(frame_delay)  # Adjust speed of playback

    vis.destroy_window()

if __name__ == "__main__":
    # Set the folder containing PCD frames
    pcd_folder = "/Users/pradeeppatil/workspace/python-nik/take4/"
    
    # Play PCD sequence with rotated view
    play_pcd_sequence(pcd_folder, frame_delay=0.1)  # Adjust frame_delay to control speed