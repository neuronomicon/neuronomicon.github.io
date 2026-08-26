# Voxelyze-Unity System User Manual (v0.5)

**※ WARNING:** This architecture manually controls the lock-step synchronization of the agents via scripts. Therefore, **you must NEVER use the `Decision Requester` component provided by Unity ML-Agents.**

### Step 1: Global Manager Setup
Create a central command center that controls the timing of all simulations, communication with the C++ DLL, and toggles the ML-Agents mode globally.

1. In the Unity Hierarchy window, create a new **Empty GameObject** and name it `VoxelEngineManager`.
2. Add the following **three core scripts** to this object in order (Add Component):
   * `VoxelEngineCore.cs`
   * `VoxelRobotBuilder.cs`
   * `VoxelRLManager.cs`
3. **VoxelEngineCore Settings:** 
   * `Dll Folder Path`: Enter the absolute path to the directory containing your built C++ DLL file. (Alternatively, use the custom editor menu `Voxel Engine > Select DLL File` to set this automatically).
   * `Is Ml Agent`: **(Crucial)** This single checkbox toggles the entire scene between Training (RL) mode (ON) and pure Rendering/Physics Test mode (OFF). Ensure this is checked for training.
4. **VoxelRLManager Settings:** 
   * `Steps Per Simulation Cycle`: The number of micro-steps the C++ engine computes per call (e.g., 20).
   * `Custom Decision Period`: The AI's decision-making frequency (e.g., 25). This variable centrally forces the action cycle for all robots.
   * `Action Size Per Robot`: The total number of actions the neural network outputs (e.g., 4 motors × 2 parameters = 8).

### Step 2: Training Environment & Obstacle Setup
Set up the isolated stage where each robot will act independently.

1. Create a new **Empty GameObject** in the scene and name it `TrainingEnvironment_0`.
2. Attach the `VoxelPhysicsManager.cs` script to this object. (This relays rigid-body collision forces within the training environment).
3. Add the following elements as **Child objects** of `TrainingEnvironment_0`:
   * **Robot Folder:** Create an empty object and name it `VoxelRobot_0`.
   * **Terrain:** Create a `3D Object -> Plane` to act as the floor.
   * **Colliders (Optional):** Create interactive objects like Cubes or Spheres. (These MUST have a `Rigidbody` and a `Box/Sphere Collider` attached).
4. **Register Colliders:** Drag and drop the newly created collider objects into the `Interactive Objects` array in the `VoxelPhysicsManager` inspector.

### Step 3: Individual Robot Instance Setup
Configure the actual robot object where voxels will be rendered and the AI will reside.

1. Select the `VoxelRobot_0` object created earlier and attach the following scripts:
   * `VoxelGraphicRenderer.cs`
   * `VoxelPhysicsInfo.cs`
   * `VoxelRobotAgent.cs` 
2. When you attach `VoxelRobotAgent`, Unity will automatically attach the `Behavior Parameters` component alongside it.
3. **(Mandatory Check)** If the `Decision Requester` component is attached, you **must delete it (Remove Component)**. Action requests are handled manually by the `VoxelRLManager`.

### Step 4: Target Setup (Optional)
If you are training for target tracking tasks, set up a target object.

1. As a child of `TrainingEnvironment_0`, create a `3D Object -> Sphere` and rename it to `Target`.
2. Apply a distinct, highly visible material (e.g., bright red) so it's easily recognizable.

### Step 5: Task Profile (ScriptableObject) Creation & Link
Create an asset that defines the robot's brain specifications (observation size, action size, morphology).

1. In the Project window, right-click and select `Create -> RL Tasks -> Target Tracking Profile` to create a new asset file.
2. Click the created asset. In the Inspector, you will see configuration values (Space Size, Continuous Actions, etc.) protected by `[ReadOnly]`. These values are hardcoded internally based on your task design to prevent accidental misconfigurations.
3. Select the `VoxelRobot_0` object in the scene again.
4. **Drag and drop the created asset file** into the **Task Profile** slot on the `VoxelRobotAgent` component.
5. Once assigned, a cleanly organized `Runtime State` section will expand below it. **Drag and drop the `Target` object (from Step 4) into the `Target Transform` slot** located there. (Internal calculation variables are cleanly hidden via `[HideInInspector]`).

### Step 6: Synchronization Check & Execution
Perform a final visual check to ensure the ML-Agents parameters match the profile's specifications. (Applying the profile usually overwrites these automatically).

1. In the `Behavior Parameters` component of `VoxelRobot_0`, visually verify the following:
   * **Vector Observation -> Space Size:** Check that this matches the observation count defined in your `TargetTrackingProfile`.
   * **Actions -> Continuous Actions:** Check that this matches the value set in the manager during Step 1.
2. If you want a 10-second episode (e.g., 10,000 C++ steps), the agent's decision count (Max Step) should be configured as: 10,000 steps / 500 steps (1 decision cycle) = **20**.

**🎉 All preparations are complete!**
*   Press the **Play (▶)** button at the top. The system will intelligently auto-launch into either Training (RL) mode or Smooth Rendering (Test) mode based on your `Is Ml Agent` checkbox.
*   For massive Multi-Agent training, simply duplicate (Ctrl+D) the entire `TrainingEnvironment_0` GameObject as many times as you want (e.g., 16 times). This will immediately initiate parallel training, fully utilizing 100% of your 128 threads!
