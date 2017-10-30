<template>
  <div id="occupancy-map">
    <b-button-group id="occupancy-map-buttons">
      <b-button @click="userGoalToggled = !userGoalToggled" :variant="userGoalToggled ? 'success' : 'secondary'"><i class="fa fa-map-marker" aria-hidden="true"></i></b-button>
    </b-button-group>
    <resize-observer @notify="resizeCanvas" />
  </div>
</template>

<script>
/* global ROSLIB, ROS3D, THREE */

function get3DPositionFromClickEvent (e, camera) {
  var vector = new THREE.Vector3(e.offsetX / e.target.clientWidth * 2 - 1,
                                 -e.offsetY / e.target.clientHeight * 2 + 1,
                                 0.5)

  new THREE.Projector().unprojectVector(vector, camera)

  var dir = new THREE.Raycaster(camera.position.clone(), vector.sub(camera.position).normalize()).ray.direction

  var distance = -camera.position.z / dir.z

  return camera.position.clone().add(dir.multiplyScalar(distance))
}

export default {
  props: {
    ros: {
      required: true,
      type: Object
    },
    mapTopicName: {
      type: String,
      required: true
    },
    polygonTopicName: {
      type: String,
      required: true
    }
  },
  data () {
    return {
      viewer: null,
      tfClient: null,
      gridClient: null,
      polygon: null,
      userGoal: null,
      userGoalToggled: false
    }
  },
  watch: {
    mapTopicName () {
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

    // Setup a client to listen to TFs.
    this.tfClient = new ROSLIB.TFClient({
      ros: this.ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: 'map'
    })

    this.gridClient = new ROS3D.OccupancyGridClient({
      ros: this.ros,
      rootObject: this.viewer.scene,
      continuous: true,
      topic: this.mapTopicName,
      opacity: 0.5
    })

    this.polygon = new ROS3D.Polygon({
      ros: this.ros,
      rootObject: this.viewer.scene,
      topic: this.polygonTopicName,
      tfClient: this.tfClient
    })

    var sphereGeometry = new THREE.SphereGeometry(0.5, 100, 100)
    var sphereMaterial = new THREE.MeshBasicMaterial({color: 0xffff00})
    this.userGoal = new THREE.Mesh(sphereGeometry, sphereMaterial)
    this.userGoal.position.x = 1e9
    this.viewer.scene.add(this.userGoal)

    this.resizeCanvas()

    this.viewer.renderer.domElement.addEventListener('click', (e) => {
      if (this.userGoalToggled) {
        var pos = get3DPositionFromClickEvent(e, this.viewer.camera)
        this.userGoal.position = pos
        this.userGoalToggled = false
      }
    }, false)
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
  position: relative;
}
#occupancy-map-buttons {
  position: absolute;
  top: 10px;
  left: 10px;
}
</style>
