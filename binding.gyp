{
	"targets": [
		{
			"target_name": "recastnavigation",

			"include_dirs": [
				"./SDL",
				"./recastnavigation/DebugUtils/Include",
				"./recastnavigation/Detour/Include",
				"./recastnavigation/DetourCrowd/Include",
				"./recastnavigation/DetourTileCache/Include",
				"./recastnavigation/Recast/Include/",
				"./recastnavigation/RecastDemo/Include/",
				"<!(node -e \"require('nan')\")"
			],

			"sources": [
				"./recastnavigation/DebugUtils/Source/DebugDraw.cpp",

				"./recastnavigation/Recast/Source/Recast.cpp",
				"./recastnavigation/Recast/Source/RecastAlloc.cpp",
				"./recastnavigation/Recast/Source/RecastArea.cpp",
				"./recastnavigation/Recast/Source/RecastAssert.cpp",
				"./recastnavigation/Recast/Source/RecastContour.cpp",
				"./recastnavigation/Recast/Source/RecastFilter.cpp",
				"./recastnavigation/Recast/Source/RecastLayers.cpp",
				"./recastnavigation/Recast/Source/RecastMesh.cpp",
				"./recastnavigation/Recast/Source/RecastMeshDetail.cpp",
				"./recastnavigation/Recast/Source/RecastRasterization.cpp",
				"./recastnavigation/Recast/Source/RecastRegion.cpp",

				"./recastnavigation/Detour/Source/DetourAlloc.cpp",
				"./recastnavigation/Detour/Source/DetourAssert.cpp",
				"./recastnavigation/Detour/Source/DetourCommon.cpp",
				"./recastnavigation/Detour/Source/DetourNavMesh.cpp",
				"./recastnavigation/Detour/Source/DetourNavMeshBuilder.cpp",
				"./recastnavigation/Detour/Source/DetourNavMeshQuery.cpp",
				"./recastnavigation/Detour/Source/DetourNode.cpp",
				
				"./recastnavigation/DetourCrowd/Source/DetourProximityGrid.cpp",
				"./recastnavigation/DetourCrowd/Source/DetourCrowd.cpp",
				"./recastnavigation/DetourCrowd/Source/DetourLocalBoundary.cpp",
				"./recastnavigation/DetourCrowd/Source/DetourObstacleAvoidance.cpp",
				"./recastnavigation/DetourCrowd/Source/DetourPathCorridor.cpp",
				"./recastnavigation/DetourCrowd/Source/DetourPathQueue.cpp",
				
				"./recastnavigation/RecastDemo/Source/Sample.cpp",
				"./recastnavigation/RecastDemo/Source/SampleInterfaces.cpp",

				"./src/main.cc"
			],
		}
	]
}