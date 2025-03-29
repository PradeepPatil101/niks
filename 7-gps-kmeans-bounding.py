import open3d as o3d
import os
import time
import natsort  # To sort filenames naturally
import numpy as np
from sklearn.cluster import KMeans

def remove_ground_plane(pcd, distance_threshold=0.4, ransac_n=3, num_iterations=1000):
    """Remove ground plane using RANSAC."""
    plane_model, inlier_indices = pcd.segment_plane(distance_threshold=distance_threshold,
                                                    ransac_n=ransac_n,
                                                    num_iterations=num_iterations)
    outlier_cloud = pcd.select_by_index(inlier_indices, invert=True)
    return outlier_cloud  

def apply_kmeans_clustering(pcd, n_clusters=5):
    """Apply K-Means clustering to the given point cloud."""
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        return pcd  # Return original point cloud if empty
    
    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
    labels = kmeans.fit_predict(points)

    colors = np.random.uniform(0, 1, size=(n_clusters, 3))
    colored_points = np.array([colors[label] for label in labels])

    pcd.colors = o3d.utility.Vector3dVector(colored_points)
    return pcd

def filter_points_within_bounds(pcd, boundsx=(-50, 50), boundsy=(-100,100), boundsz=(-50,50)):
    """Filter points within ±10 in X, Y, and Z."""
    points = np.asarray(pcd.points)
    mask = ((boundsx[0] <= points[:, 0]) & (points[:, 0] <= boundsx[1]) &
            (boundsy[0] <= points[:, 1]) & (points[:, 1] <= boundsy[1]) &
            (boundsz[0] <= points[:, 2]) & (points[:, 2] <= boundsz[1]))

    filtered_pcd = pcd.select_by_index(np.where(mask)[0])
    return filtered_pcd

def play_pcd_sequence(pcd_folder, frame_delay=0.1, n_clusters=5):
    pcd_files = [f for f in os.listdir(pcd_folder) if f.endswith('.pcd')]
    pcd_files = natsort.natsorted(pcd_files)
    
    if not pcd_files:
        print("No .pcd files found in the folder.")
        return

    print(f"Playing {len(pcd_files)} point cloud frames...")

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20.0)
    vis.add_geometry(coordinate_frame)

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

    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[0]))
    pcd = remove_ground_plane(pcd)
    pcd.transform(rotation_matrix)
    pcd.transform(rotation_matrix_2)
    pcd = filter_points_within_bounds(pcd)  # Apply bounding box filter
    pcd = apply_kmeans_clustering(pcd, n_clusters=n_clusters)
    vis.add_geometry(pcd)

    for pcd_file in pcd_files:
        pcd_path = os.path.join(pcd_folder, pcd_file)
        new_pcd = o3d.io.read_point_cloud(pcd_path)

        if new_pcd.is_empty():
            print(f"Skipping empty PCD: {pcd_file}")
            continue

        print(f"Displaying: {pcd_file}")

        new_pcd = remove_ground_plane(new_pcd)
        new_pcd.transform(rotation_matrix)
        new_pcd.transform(rotation_matrix_2)
        new_pcd = filter_points_within_bounds(new_pcd)  # Apply bounding box filter
        new_pcd = apply_kmeans_clustering(new_pcd, n_clusters=n_clusters)

        pcd.points = new_pcd.points
        pcd.colors = new_pcd.colors
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        time.sleep(frame_delay)

    vis.destroy_window()

if __name__ == "__main__":
    pcd_folder = "/Users/pradeeppatil/workspace/python-nik/take4/"
    play_pcd_sequence(pcd_folder, frame_delay=0.5, n_clusters=5)
