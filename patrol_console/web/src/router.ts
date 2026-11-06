// 页面路由。meta.titleKey 是菜单标题在语言包里的键。
// patrol_console 裁剪版：去掉 maps / navigation / evaluation（对应后端模块已裁掉），
// 只留建图（点云可视化 + 遥控）和系统监控。
import { createRouter, createWebHashHistory, type RouteRecordSingleView } from 'vue-router'
import MonitorView from '@/views/MonitorView.vue'
import MappingView from '@/views/MappingView.vue'

interface PageMeta {
  titleKey: string
  icon: string
}

export const pages: (RouteRecordSingleView & { meta: PageMeta })[] = [
  { path: '/mapping', name: 'mapping', component: MappingView, meta: { titleKey: 'nav.mapping', icon: 'MapLocation' } },
  { path: '/monitor', name: 'monitor', component: MonitorView, meta: { titleKey: 'nav.monitor', icon: 'Monitor' } },
]

export const router = createRouter({
  // hash 模式：地址形如 /#/monitor，后端只需提供 index.html，刷新页面不会 404
  history: createWebHashHistory(),
  routes: [{ path: '/', redirect: '/mapping' }, ...pages],
})
