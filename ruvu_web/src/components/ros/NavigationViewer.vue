<template>
  <div id="occupancy-map">
    <resize-observer @notify="resizeCanvas" />
  </div>
</template>

<script>
/* global ROS3D, THREE */

export default {
  props: {
    ros: {
      required: true,
      type: Object
    },
    topicName: {
      type: String,
      required: true
    }
  },
  data () {
    return {
      viewer: null,
      gridClient: null
    }
  },
  watch: {
    topicName () {
      // this.readvertise()
    }
  },
  mounted () {
    this.viewer = new ROS3D.Viewer({
      divID: 'occupancy-map',
      width: 1,
      height: 1,
      antialias: true,
      cameraPose: {
        x: 0,
        y: 0,
        z: 10
      }
    })
    this.viewer.cameraControls.userRotateSpeed = 0
    this.viewer.camera.lookAt(new THREE.Vector3(5, 5, 0))
    this.viewer.cameraControls.showAxes = function () {}
    window.VIEWER = this.viewer
    this.gridClient = new ROS3D.OccupancyGridClient({
      ros: this.ros,
      rootObject: this.viewer.scene,
      continuous: true
    })
    this.resizeCanvas()
  },
  methods: {
    resizeCanvas () {
      var parentElement = this.viewer.renderer.domElement.parentElement
      this.viewer.camera.aspect = parentElement.offsetWidth / parentElement.offsetHeight
      this.viewer.camera.updateProjectionMatrix()
      this.viewer.renderer.setSize(parentElement.offsetWidth, parentElement.offsetHeight)
    }
  }
}
</script>

<style>
#occupancy-map {
  width: 100%;
  height: 100%;
}
</style>
