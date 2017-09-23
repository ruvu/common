<template>
  <div class="card">
    <div class="card-header">
      Twist Teleop
    </div>
    <div class="card-block twist-teleop-canvas-container">
      <b-input-group>
        <b-input-group-addon>Topic name</b-input-group-addon>
        <b-form-input v-model="topicName"
                      placeholder="Please enter a topic"></b-form-input>
      </b-input-group>
      <twist-teleop-canvas :topicName="topicName"></twist-teleop-canvas>
    </div>
    <div class="card-footer text-muted">
      Publishing to {{topicName}}
    </div>
  </div>
</template>

<script>
import TwistTeleopCanvas from '@/components/ros/TwistTeleopCanvas'

export default {
  components: {
    TwistTeleopCanvas
  },
  data () {
    return {
      topicName: 'cmd_vel'
    }
  }
}
</script>

<style>
twist-teleop-canvas-container {
  width: 100%;
  height: 600px;
}
</style>
