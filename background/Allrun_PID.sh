#!/bin/sh
#SBATCH --account=eleceng
#SBATCH --partition=ada
#SBATCH --nodes=1
#SBATCH --ntasks=40
#SBATCH --time=4-00:00:00  # Requesting just under the maximum allowed time
#SBATCH --job-name="PID_test"
#SBATCH --mail-user=nkhadi001@myuct.ac.za
#SBATCH --mail-type=BEGIN,END,FAIL

module load mpi/openmpi-4.0.1
module load python/miniconda3-py38  # Load the correct Python module version

# OpenFOAM
SINGULARITY_PATH=/opt/exp_soft/singularity-containers
FOAM_BASHRC=/opt/OpenFOAM/OpenFOAM-v2106/etc/bashrc

caseDir=/home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/background
pidControlScript=/home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/thirdParty/MoorDyn.build/wrappers/python/moordyn/moordyn_PID_Control.py

# Ensure the case directory exists
if [ ! -d "$caseDir" ]; then
    echo "Directory $caseDir does not exist. Creating it now."
    mkdir -p $caseDir || { echo "Failed to create directory $caseDir"; exit 1; }
fi

# Ensure correct directory for mergeMeshes
cd $caseDir || { echo "Failed to change directory to $caseDir"; exit 1; }

# Run OpenFOAM commands with error checking
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif surfaceFeatureExtract -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif snappyHexMesh -overwrite -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1

singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif surfaceFeatureExtract -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif snappyHexMesh -overwrite -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2

singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh -case $caseDir || { echo "blockMesh failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif mergeMeshes $caseDir /home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1 -overwrite || { echo "mergeMeshes overset1 failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif mergeMeshes $caseDir /home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2 -overwrite || { echo "mergeMeshes overset2 failed"; exit 1; }

# Refine free surface area
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif refine topoSet -case $caseDir system/topoSetDict.refine || { echo "toposetDict.refine failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif refineMesh -case $caseDir system/refineMeshDict -overwrite || { echo "refineMeshDict failed"; exit 1; }

singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif renumberMesh -constant -overwrite -case $caseDir || { echo "renumbermesh failed"; exit 1; }

singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case $caseDir -dict system/topoSetDictBox1 || { echo "topoSetDictBox1 failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case $caseDir -dict system/topoSetDictBox2 || { echo "topoSetDictBox2 failed"; exit 1; }
cp -r $caseDir/0.orig $caseDir/0 || { echo "Failed to copy initial conditions"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif setFields -case $caseDir || { echo "setFields failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif decomposePar -case $caseDir || { echo "decomposePar failed"; exit 1; }

singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif mpirun -np 32 overInterDyMFoam -parallel -case $caseDir > $caseDir/solver.log 2>&1 &
OVERINTERDYMFOAM_PID=$!

sleep 5  # Ensure some delay for proper initialization, adjust as needed 

# Run the Python PID control script and log output

cd /home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/thirdParty/MoorDyn.build/wrappers/python/moordyn/
python3 moordyn_PID_Control.py > $caseDir/pid_control.log 2>&1 &
PID_CONTROL_PID=$!

# Wait for both processes to complete
wait $OVERINTERDYMFOAM_PID
wait $PID_CONTROL_PID

# Check if the PID control log file was created and is not empty
if [ ! -s $caseDir/pid_control.log ]; then
    echo "Warning: pid_control.log is missing or empty."
fi

# Check if the solver log file was created and is not empty
if [ ! -s $caseDir/solver.log ]; then
    echo "Warning: solver.log is missing or empty."
fi

# Get the available time directories
timeDirs=$(ls -d $caseDir/processor0/[0-9]* | xargs -n 1 basename)

# Ensure that there are time directories to reconstruct
if [ -z "$timeDirs" ]; then
    echo "No time directories found for reconstruction."
    exit 1
fi

# Perform reconstruction
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif reconstructPar -case $caseDir -time $timeDirs || { echo "reconstructPar failed"; exit 1; }

# Clean up processor directories
# rm -r $caseDir/processor* || { echo "Failed to remove processor directories"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif touch Mooredvessel_waves.foam -case $caseDir || { echo "Failed to create foam file"; exit 1; }

echo "Job completed successfully."
