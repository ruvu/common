import Vue from 'vue'
import Router from 'vue-router'

import TwistTeleop from '@/components/TwistTeleop'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'TwistTeleop',
      component: TwistTeleop
    }
  ]
})
