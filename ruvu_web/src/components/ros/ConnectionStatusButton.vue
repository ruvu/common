<template>
  <span>
    <b-button :variant="variant"
               v-text="status.message"
               id="connectionStatusButton"></b-button>
    <b-tooltip target="connectionStatusButton" placement="bottom">
      <table>
        <tr>
          <td>url:</td>
          <td v-text="url"></td>
        </tr>
        <tr v-if="status.connectionStartTime">
          <td>since:</td>
          <td>{{status.connectionStartTime | moment("YYYY/MM/DD hh:mm")}}</td>
        </tr>
      </table>
    </b-tooltip>
  </span>
</template>

<script>
import TeleopCanvas from '@/components/ros/TeleopCanvas'

import ros from '@/services/ros'

export default {
  components: {
    TeleopCanvas
  },
  data () {
    return {
      status: ros.status,
      url: ros.url
    }
  },
  computed: {
    variant () {
      var statusVariantMapping = {
        'connecting': 'outline-warning sm',
        'connected': 'outline-success sm',
        'closed': 'outline-danger sm'
      }
      return statusVariantMapping[this.status.message]
    }
  }
}
</script>

<style>

</style>
