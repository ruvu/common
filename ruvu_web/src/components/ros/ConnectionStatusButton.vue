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
        <tr v-if="connectionLifetime">
          <td>lifetime:</td>
          <td v-text="connectionLifetime"></td>
        </tr>
      </table>
    </b-tooltip>
  </span>
</template>

<script>
import TeleopCanvas from '@/components/ros/TeleopCanvas'

import ros from '@/services/ros'
import moment from 'moment'

export default {
  components: {
    TeleopCanvas
  },
  mounted () {
    setInterval(() => {
      this.connectionLifetime = moment(this.status.connectionStartTime).toNow(true)
    }, 1000)
  },
  data () {
    return {
      status: ros.status,
      url: ros.url,
      connectionLifetime: ''
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
