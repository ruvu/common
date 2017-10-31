<template>
  <div id="occupancy-map">
    <div id="occupancy-map-buttons">
      <b-button @click="robotTracking = !robotTracking" :variant="robotTracking ? 'success' : 'secondary'">
        <i class="fa fa-crosshairs" aria-hidden="true"></i>
      </b-button>
      <b-button-group>
        <b-button @click="userGoalToggled = !userGoalToggled" :variant="userGoalToggled ? 'success' : 'secondary'">
          <i class="fa fa-map-marker" aria-hidden="true"></i>
        </b-button>
        <b-button @click="userGoalToggled = false; userGoal.position.x = 1e9" v-if="userGoalToggled">
          <i class="fa fa-remove" aria-hidden="true"></i>
        </b-button>
      </b-button-group>
    </div>
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
    userGoalTopicName: {
      type: String,
      required: true
    },
    robotFrameId: {
      type: String,
      required: true
    },
    pixelFactor: {
      type: Number,
      default: 4
    }
  },
  data () {
    return {
      viewer: null,
      tfClient: null,
      gridClient: null,
      robot: null,
      robotTracking: false,
      userGoal: null,
      userGoalToggled: false,
      userGoalPublisher: null
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
    window.VIEWER = this.viewer

    // Setup a client to listen to TFs.
    this.tfClient = new ROSLIB.TFClient({
      ros: this.ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: 'map'
    })

    var that = this
    this.tfClient.subscribe(this.robotFrameId, (e) => {
      if (this.robotTracking) {
        // that.viewer.camera.position.x = e.translation.x
        // that.viewer.camera.position.y = e.translation.y
      }
      that.robot.position.x = e.translation.x
      that.robot.position.y = e.translation.y
    })

    this.gridClient = new ROS3D.OccupancyGridClient({
      ros: this.ros,
      rootObject: this.viewer.scene,
      continuous: true,
      topic: this.mapTopicName,
      opacity: 0.5
    })

    var robotGeometry = new THREE.SphereGeometry(0.3, 100, 100)
    var robotMaterial = new THREE.MeshBasicMaterial({color: 0xcc00ff})
    this.robot = new THREE.Mesh(robotGeometry, robotMaterial)
    this.robot.position.x = 1e9
    this.viewer.scene.add(this.robot)

    var sphereGeometry = new THREE.SphereGeometry(0.3, 100, 100)
    var sphereMaterial = new THREE.MeshBasicMaterial({color: 0xffff00})
    this.userGoal = new THREE.Mesh(sphereGeometry, sphereMaterial)
    this.userGoal.position.x = 1e9
    this.viewer.scene.add(this.userGoal)

    this.userGoalPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: this.userGoalTopicName,
      messageType: 'geometry_msgs/PoseStamped'
    })

    this.resizeCanvas()

    this.viewer.renderer.domElement.addEventListener('click', (e) => {
      if (this.userGoalToggled) {
        var pos = get3DPositionFromClickEvent(e, this.viewer.camera)
        this.userGoal.position = pos
        this.userGoalToggled = false
        this.sendUserGoal(pos)
      }
    }, false)
  },
  methods: {
    resizeCanvas () {
      var parentElement = this.viewer.renderer.domElement.parentElement
      this.viewer.camera.aspect = parentElement.offsetWidth / parentElement.offsetHeight
      this.viewer.camera.updateProjectionMatrix()
      this.viewer.renderer.setSize(parentElement.offsetWidth / this.pixelFactor, parentElement.offsetHeight / this.pixelFactor)
      this.viewer.renderer.domElement.style = 'width: 100%; height: 100%'
    },
    sendUserGoal (position) {
      this.userGoalPublisher.publish(new ROSLIB.Message({
        pose: {
          position: position
        }
      }))
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
