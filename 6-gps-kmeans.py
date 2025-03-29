import open3d as o3d
import os
import time
import natsort  # To sort filenames naturally
import numpy as np
from sklearn.cluster import KMeans  # Import K-Means clustering

def remove_ground_plane(pcd, distance_threshold=0.3, ransac_n=10, num_iterations=1000):
    # Perform ground plane segmentation using RANSAC
    plane_model, inlier_indices = pcd.segment_plane(distance_threshold=distance_threshold,
                                                   ransac_n=ransac_n,
                                                   num_iterations=num_iterations)

    # Extract the inliers (ground points) and outliers (non-ground points)
    outlier_cloud = pcd.select_by_index(inlier_indices, invert=True)
    
    return outlier_cloud  # This is the point cloud with the ground removed

def apply_kmeans_clustering(pcd, n_clusters=5):
    """Apply K-Means clustering to the given point cloud."""
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        return pcd  # Return original point cloud if empty
    
    # Apply K-Means clustering
    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
    labels = kmeans.fit_predict(points)

    # Assign colors based on cluster labels
    colors = np.random.uniform(0, 1, size=(n_clusters, 3))
    colored_points = np.array([colors[label] for label in labels])

    pcd.colors = o3d.utility.Vector3dVector(colored_points)
    
    return pcd

def play_pcd_sequence(pcd_folder, frame_delay=0.1, n_clusters=5):
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

    # Load first frame and remove the ground plane
    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[0]))
    pcd = remove_ground_plane(pcd)  # Remove the ground plane
    pcd = apply_kmeans_clustering(pcd, n_clusters=n_clusters)  # Apply K-Means clustering
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

        # Remove the ground from the new frame
        new_pcd = remove_ground_plane(new_pcd)

        # Apply K-Means clustering
        new_pcd = apply_kmeans_clustering(new_pcd, n_clusters=n_clusters)

        # Apply the same rotation to the new frame
        new_pcd.transform(rotation_matrix)
        new_pcd.transform(rotation_matrix_2) 

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
    
    # Play PCD sequence with rotated view, ground removal, and K-Means clustering
    play_pcd_sequence(pcd_folder, frame_delay=0.1, n_clusters=50)  # Adjust frame_delay & clusters
