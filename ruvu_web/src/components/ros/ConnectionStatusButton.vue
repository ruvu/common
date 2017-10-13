<template>
  <span>
    <b-button :variant="ros.isConnected ? 'outline-success sm' : 'outline-danger sm'"
               v-text="ros.isConnected ? 'connected' : 'closed'"
               id="connectionStatusButton"></b-button>
    <b-tooltip target="connectionStatusButton" placement="bottom">
      <table>
        <tr>
          <td>url:</td>
          <td v-text="ros.socket.url"></td>
        </tr>
      </table>
    </b-tooltip>
  </span>
</template>

<script>
import TeleopCanvas from '@/components/ros/TeleopCanvas'

export default {
  components: {
    TeleopCanvas
  },
  props: {
    ros: {
      required: true,
      type: Object
    }
  }
}
</script>

<style>

</style>
