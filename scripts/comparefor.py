import cv2
import cv2.aruco as aruco
import numpy as np

# Define the known size of the ArUco marker in the real world (in meters)
aruco_marker_size = 0.48 # Example: 10 cm (adjust this to match your markers)

# image1 = cv2.imread("/home/rafidahmadd/catkin_ws/src/drone_pkg/image_data/base2.jpg")
# image2 = cv2.imread("/home/rafidahmadd/catkin_ws/src/drone_pkg/image_data/Image_Bener/baseuav0titikB.jpg")
# image1 = cv2.imread("/home/rafidahmadd/Pictures/captured/aruco_base.png")
# image2 = cv2.imread("/home/rafidahmadd/Pictures/captured/aruco_down1mm.png")
# image1 = cv2.imread("/home/rafidahmadd/Pictures/captured/fromDrone/reference.png")
# image2 = cv2.imread("/home/rafidahmadd/Pictures/captured/fromDrone/move4.png")
image1 = cv2.imread("/home/rafidahmadd/Pictures/captured/fromDrone/reference4.jpg")
image2 = cv2.imread("/home/rafidahmadd/Pictures/captured/fromDrone/move13.jpg")
# image1 = cv2.imread("/home/rafidahmadd/Pictures/captured/fromDrone/captured_image1.png")
# image2 = cv2.imread("/home/rafidahmadd/Pictures/captured/fromDrone/captured_image2.png")

# Check if the images are loaded successfully and not empty
if image1 is None or image2 is None:
    print("Error: Failed to load one or both images.")
else:
    # Define the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

    # Detect ArUco markers in both photos
    corners1, ids1, _ = aruco.detectMarkers(image1, aruco_dict)
    corners2, ids2, _ = aruco.detectMarkers(image2, aruco_dict)

    if ids1 is not None and ids2 is not None:
        # Create a list to store marker correspondences and their displacements in meters
        marker_displacements = []

        # Iterate through marker IDs to match and calculate displacements
        for id in np.intersect1d(ids1, ids2):
            idx1 = np.where(ids1 == id)[0]
            idx2 = np.where(ids2 == id)[0]

            if len(idx1) > 0 and len(idx2) > 0:
                corner1 = corners1[idx1[0]]
                corner2 = corners2[idx2[0]]

                centroid1 = corner1[0].mean(axis=0)
                centroid2 = corner2[0].mean(axis=0)

                # Calculate the pixel-based displacement
                displacement_pixels = (centroid2[0] - centroid1[0], centroid2[1] - centroid1[1])

                # Calculate the displacement in meters using the known size
                displacement_meters = (
                    displacement_pixels[0] * (aruco_marker_size *2/ corner1[0].max()),
                    displacement_pixels[1] * (aruco_marker_size *2/ corner2[0].max())
                )

                # ((corner1[0].mean() + corner1[0].max()) /2)
                # print(corner1[0].max())
                # print(corner2[0].max())
                # print(corner1[0].mean())
                # print(corner2[0].mean())
                # print(corner1[0])
                # print(corner2[0])

                marker_displacements.append((id, displacement_meters))

        # Visualize the results
        for id, displacement in marker_displacements:
            idx1 = np.where(ids1 == id)[0]
            idx2 = np.where(ids2 == id)[0]

            if len(idx1) > 0 and len(idx2) > 0:
                cv2.aruco.drawDetectedMarkers(image1, [corners1[int(idx1)]])
                cv2.aruco.drawDetectedMarkers(image2, [corners2[int(idx2)]])
                # cv2.arrowedLine(image1, (int(centroid1[0]), int(centroid1[1])), (int(centroid1[0] + displacement[0]), int(centroid1[1] + displacement[1])), (0, 0, 255), 2)
                cv2.arrowedLine(image1, (int(centroid1[0]), int(centroid1[1])), (int(centroid1[0] + displacement_pixels[0]), int(centroid1[1] + displacement_pixels[1])), (0, 0, 255), 2)
                # cv2.arrowedLine(image1, (int(centroid1[1])), (int(centroid1[0] + displacement[0]), int(centroid1[1] + displacement[1])), (0, 0, 255), 2)
                # cv2.arrowedLine(image1, centroid1, (centroid1[0] + displacement[0], centroid1[1] + displacement[1]), (0, 0, 255), 2)

        # Display the images with visualized results
        cv2.imshow('Reference', image1)
        cv2.imshow('Moved Image1', image2)
        print(f"PATOK ADA!!!")
        
        # Print displacement in meters
        for id, displacement in marker_displacements:
            print(f"Marker ID {id} Displacement (X, Y) in Meters: {displacement[0]:.3f} m, {displacement[1]:.3f} m")
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
 
        # # Print displacement in meters
        # for id, displacement in marker_displacements:
        #     print(f"Marker ID {id} Displacement (X, Y) in Meters: {displacement[0]:.3f} m, {displacement[1]:.3f} m")

    else:
        print("Aruco Not Detected in image 2")
        print("Penanda Patok Hilang!!!!")
        cv2.imshow('Reference Image', image1)
        cv2.imshow('Moved Image', image2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()