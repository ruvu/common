<template>
  <div id="app">
<!--     <b-navbar-brand href="#">
      <img src="/static/logo.png" class="d-inline-block align-top" alt="RUVU">
    </b-navbar-brand> -->
    <b-navbar toggleable="md"  type="light" variant="light" :sticky="true" fixed="top">
      <!-- Right aligned nav items -->
      <b-nav is-nav-bar class="ml-auto">
        <connection-status-button :ros="ros"></connection-status-button>
      </b-nav>
    </b-navbar>

    <div id="navigation">
      <navigation-viewer :ros="ros" mapTopicName="/sergio/gmapping/map" userGoalTopicName="/move_base_simple/goal" robotFrameId="sergio/base_link"></navigation-viewer>
    </div>

    <div id="teleop">
      <twist-teleop-canvas topicName="/sergio/base/references" :ros="ros"></twist-teleop-canvas>
    </div>
  </div>
</template>

<script>
import ros from '@/services/ros'

import ConnectionStatusButton from '@/components/ros/ConnectionStatusButton'
import NavigationViewer from '@/components/ros/NavigationViewer'
import TwistTeleopCanvas from '@/components/ros/TwistTeleopCanvas'

export default {
  components: {
    ConnectionStatusButton,
    NavigationViewer,
    TwistTeleopCanvas
  },
  data () {
    return {
      ros: ros
    }
  },
  name: 'app'
}
</script>

<style>
#navigation {
  top: 54px;
  width: 100%;
  margin: 0;
  padding: 0;
  display:block;
  position:absolute;
  height: 100%;
  overflow: hidden;
}
#teleop {
  z-index: 10000;
  width: 20%;
  height: 20%;
  position: absolute;
  bottom: 0;
  left: 0;
  background-color: rgba(255, 255, 255, 0.5);
}
#app {
  overflow: hidden;
}
body {
  overflow: hidden;
}
.btn {
  cursor: pointer;
}
</style>
