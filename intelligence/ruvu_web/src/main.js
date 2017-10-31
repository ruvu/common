// The Vue build version to load with the `import` command
// (runtime-only or standalone) has been set in webpack.base.conf with an alias.
import Vue from 'vue'
import App from './NavigationApp'
import router from './router'

import ros from './services/ros'
window.ros = ros

import VueResize from 'vue-resize'
Vue.use(VueResize)

import BootstrapVue from 'bootstrap-vue'
Vue.use(BootstrapVue)

import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'
import 'font-awesome/css/font-awesome.min.css'

import VueMoment from 'vue-moment'
Vue.use(VueMoment)

import Vue2Touch from 'vue2-touch'
Vue.use(Vue2Touch)

import VueThreejs from 'vue-threejs'
Vue.use(VueThreejs)

Vue.config.productionTip = false

/* eslint-disable no-new */
new Vue({
  el: '#app',
  router,
  template: '<App/>',
  components: { App }
})
