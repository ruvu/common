<template>
  <div id="occupancy-map">
    <renderer :size="{ w: 600, h: 400 }">
      <scene>
        <camera :position="{ z: 15 }"></camera>
        <mesh :obj="mesh" :position="{ y: -200 }"></mesh>
        <animation :fn="animate" :speed="3"></animation>
      </scene>
    </renderer>
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
    }
  },
  data () {
  },
  watch: {
    mapTopicName () {
      // this.readvertise()
    }
  },
  mounted () {
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
