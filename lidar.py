import open3d as o3d
import os
import time
import natsort  # To sort filenames naturally

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

    # Define transformation matrix to flip the view (180-degree rotation around Y-axis)
    flip_transform = [[-1,  0,  0,  0],  # Flip X
                      [ 0,  1,  0,  0],  # Keep Y
                      [ 0,  0, -1,  0],  # Flip Z
                      [ 0,  0,  0,  1]]  # Homogeneous transformation

    # Load first frame
    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[0]))
    pcd.transform(flip_transform)  # Apply flip
    vis.add_geometry(pcd)

    for pcd_file in pcd_files:
        pcd_path = os.path.join(pcd_folder, pcd_file)
        new_pcd = o3d.io.read_point_cloud(pcd_path)

        if new_pcd.is_empty():
            print(f"Skipping empty PCD: {pcd_file}")
            continue

        print(f"Displaying: {pcd_file}")

        # Apply flip transformation to the new frame
        new_pcd.transform(flip_transform)

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
    
    # Play PCD sequence with flipped view
    play_pcd_sequence(pcd_folder, frame_delay=0.1)  # Adjust frame_delay to control speed
