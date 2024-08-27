#!/bin/sh
#SBATCH --account=eleceng
#SBATCH --partition=ada
#SBATCH --nodes=1
#SBATCH --ntasks=40
#SBATCH --time=4-00:00:00  # Requesting just under the maximum allowed time
#SBATCH --job-name="test1"
#SBATCH --mail-user=nkhadi001@myuct.ac.za
#SBATCH --mail-type=BEGIN,END,FAIL

module load mpi/openmpi-4.0.1

# OpenFOAM
SINGULARITY_PATH=/opt/exp_soft/singularity-containers
FOAM_BASHRC=/opt/OpenFOAM/OpenFOAM-v2106/etc/bashrc

caseDir=/home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/background

# Ensure the case directory exists
if [ ! -d "$caseDir" ]; then
    echo "Directory $caseDir does not exist. Creating it now."
    mkdir -p $caseDir || { echo "Failed to create directory $caseDir"; exit 1; }
fi

# Ensure correct directory for mergeMeshes
cd $caseDir || { echo "Failed to change directory to $caseDir"; exit 1; }

# Run OpenFOAM commands with error checking
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1
#singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif snappyHexMesh -overwrite -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1

singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2
#singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif snappyHexMesh -overwrite -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2

singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh -case $caseDir || { echo "blockMesh failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif mergeMeshes $caseDir /home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1 -overwrite || { echo "mergeMeshes overset1 failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif mergeMeshes $caseDir /home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-2 -overwrite || { echo "mergeMeshes overset2 failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case $caseDir -dict system/topoSetDictBox1 || { echo "topoSetDictBox1 failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case $caseDir -dict system/topoSetDictBox2 || { echo "topoSetDictBox2 failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case $caseDir || { echo "topoSet failed"; exit 1; }
cp -r $caseDir/0.orig $caseDir/0 || { echo "Failed to copy initial conditions"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif setFields -case $caseDir || { echo "setFields failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif decomposePar -case $caseDir || { echo "decomposePar failed"; exit 1; }
singularity run $SINGULARITY_PATH/openfoam/OpenFoam-2306-FoamMoor.sif mpirun -np 8 overInterDyMFoam -parallel -case $caseDir || { echo "overInterDyMFoam failed"; exit 1; }

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