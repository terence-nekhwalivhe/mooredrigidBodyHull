#!/bin/sh

#SBATCH --account=eleceng
#SBATCH --partition=ada
#SBATCH --nodes=1 --ntasks=40
#SBATCH --time=02:00:00
#SBATCH --job-name="rgd_overoset"
#SBATCH --mail-user=nkhadi001@myuct.ac.za
#SBATCH --mail-type=BEGIN,END,FAIL

# OpenFOAM
SINGULARITY_PATH=/opt/exp_soft/singularity-containers
FOAM_BASHRC=/opt/OpenFOAM/OpenFOAM-v2106/etc/bashrc

cd floatingBody
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif topoSet
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif snappyHexMesh -overwrite
cd ../background
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif mergeMeshes . ../floatingBody -overwrite
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif topoSet
cp -r 0.orig 0
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif setFields
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif decomposePar
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif mpirun -np 8 overInterDyMFoam -parallel
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif reconstructPar
rm -r processor*