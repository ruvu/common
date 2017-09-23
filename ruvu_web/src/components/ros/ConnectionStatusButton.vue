<template>
  <b-button :variant="variant" v-text='status.message'>Connected</b-button>
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
      status: ros.status
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
