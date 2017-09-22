import Vue from 'vue'
import Router from 'vue-router'

import CmdVelTeleop from '@/components/CmdVelTeleop'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'CmdVelTeleop',
      component: CmdVelTeleop
    }
  ]
})
