# pickhacks-2023
Curtis Brinker and Tanner May (ID: M7HSF6xBjw)

# SaferCaver

## Inspiration

Caves hold a special place in the environment. They host unique ecosystems, not to mention their importance in the understanding of geology. However, learning about cave systems can be an incredibly dangerous process, where one can get lost and find themselves in danger. In fact, every year there are 50 cave rescues performed, and unfortunately 10% result in death. Despite this, researchers and spelunkers alike still eagerly exploring these natural wonders.

However, even the most prepared and experienced can still find themselves in deep trouble deep without a means of calling for assistance. Detailed information about cave layouts are not widely available, adding to this problem. With this in mind, a system to enable safer exploration of cave systems would be incredibly beneficial. It would help cavers prepare for their trip into the the cave by providing 3D maps of  the cave, along with the capability to find the route back to the cave entrance if they were to get lost. 	 

In recent years, new technology has made LiDAR technology accessible to consumers. For example, newer iPhones and iPads now come equipped with a LiDAR sensor. We took this as inspiration to create a LiDAR based cave navigation tool to help keep cavers safe as they explore.

## What it does

Safer Caver is a proof of concept application to prepare you for your caving expedition by providing maps and by identifying your location in the cave. Using this information, the user can generate an escape plan in seconds, even if they had lost there bearings and their communication with the outside world.

The potential this has to improve caving safety can not be understated, with the potential to significantly reduce the cave rescues required, reduce the loss of life. All in all, an invaluable tool for learning about our planet's caves.  

## How we built it
Safer Caver is a series of LiDAR point cloud processing algorithms. To complete the proof of concept, these algorithms are written in Python and executed on our laptops. However, with more time and access to the appropriate hardware, it could be implemented on a LiDAR equipped mobile platform, such as an iPhone.

The application works in a series of stages:
1. Download and preprocess LiDAR point clouds of caves made available by researchers
2. Load the point cloud into the application so the user can use it
3.  Simulate a lost user in the cave
	a. Select a random location in the cave for the user to be "lost in"
	b. Sample point cloud data from that location, as if the user was scanning their surroundings
	c. Add noise to the data, to better simulate realistic scanning
4. Perform coarse alignment between the user's scanned surrounds and the previously downloaded 3D map of the cave
5. Perform a fine tuned alignment on user's surrounding by using an Iterative Closest Point (ICP) algorithm to determine the exact location the user
6. Take the location of the user and the known entrance of the cave and find the shortest route out
	a. Reformulate the problem as a graph theoretic problem
	b. Apply the A* algorithm, to find the shortest path using a Euclidean distance heuristic
	c. Take the output of the A* algorithm and display it as a path through the cave to the user

Additionally, we have created scripts to convert the LiDAR scan into 3D meshes, that are more user friendly to use.

Specifically the software libraries that we used were:
- `numpy` matrix manipulation
- `laspy` for LiDAR data processing
- `scipy` for point cloud alignment
- `open3d` for point cloud and mesh management, along with visualization

To keep our development running smoothly:
- GitHub: To keep our code version controlled as we both worked on different parts
- `pipenv`: An python virtual environment manager, to ensure that our results were reproducable on both of our machines


## Challenges we ran into
The most notable challenge that we faced in completing this is that we came up with this idea during lunch on Saturday, about half way through the hackathon. We liked the idea so much that we dropped our previous project and pivoted to this. We had to work hard to make up for lost time.

There were a lot of problems that we had to quickly resolve to manage this last minute change. The most significant was the knowledge gap. We had to do some digging to find publicly accessible datasets that fit our needs. We eventually found a great dataset consisting of scans of several Brazilian caves.

Another challenge that we faced is that we had to find a whole new set of tools last minute, and get the integrated. A lot easier said than done. There's a lot of software packages that handle LiDAR in some way, shape or form, but it was hard to find a package that did all the things that we wanted that integrated with our other tools.

Another aspect that we needed to consider is that LiDAR datasets can be very large, a scan of a single cave can be several gigabytes and consist of 50-100 million points. This level of detail is great, but it can cause other problems, and we needed to work to keep our complexity down. We were able to down sample the maps while preserving the information that we needed, reducing the space by over 75%. This allowed for our algorithms to run faster, which is very important for the future goal of mobile deployment.

## Accomplishments that we're proud of
It works! But not only that, but we finished it on such a short timeline. Hackathons are short enough, but this was another level.

Besides that, we are really proud because we see this project as truly having a future where it could get meaningful real-world use. All it would take is some more time to fine tune the algorithm parameters and transfer the code into a mobile computing environment.

## What we learned
We had a chance to work closely with LiDAR, LiDAR algorithms, and connecting those problems with software development techniques. We had a chance to try out some tools to keep our development environments in sync, which worked pretty well.

## What's next for Safer Caver
There is so much we could do with this idea, we haven't even scratched the surface. Here's just a few examples of things we've already thought about:
1. Implement the application as a mobile app. The obvious choice would be on iOS, as some iPhones and iPads have access to LiDAR sensors.
2. Generate QR codes that could be posted by cave entrances, providing links to 3D maps before entering a new cave
3. Use the QR code system as a 'check-in' and 'check-out' system for caves. If someone were to check in and not check out for a while, emergency responders could be automatically alerted of the situation
4. Allow users to scan caves and upload their maps, allowing crowd sourced maps for caves. This could generate data at a scale that regular research could not match.
5. Augment maps with hazard data so that cavers would be more prepared of dangers they may encounter.

## References

Cave dataset: https://figshare.com/articles/dataset/LiDAR_Datasets_for_Hypogenic_and_Epigenic_Caves_in_Bahia_Brazil_2019_/16864147

