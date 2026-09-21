// Shared Sidebar Navigation (SPA: navega por hash, mantém serial aberta)
(function() {
  const navItems = [
    { id: 'dashboard', href: 'dvs_dashboard.html#dashboard', icon: 'dashboard', label: 'Dashboard' },
    { id: 'calib', href: 'dvs_dashboard.html#calib', icon: 'calibration', label: 'Calibração' },
    { id: 'audio', href: 'dvs_dashboard.html#audio', icon: 'audio', label: 'Testes de Áudio' },
    { id: 'wifi', href: 'dvs_dashboard.html#wifi', icon: 'wifi', label: 'WiFi' }
  ];

  const icons = {
    dashboard: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/></svg>`,
    calibration: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M12 6v6l4 2"/><path d="M12 18v-6"/></svg>`,
    audio: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 18V5l12-2v13"/><circle cx="6" cy="18" r="3"/><circle cx="18" cy="16" r="3"/></svg>`,
    wifi: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12.55a11 11 0 0 1 14.08 0"/><path d="M1.42 9a16 16 0 0 1 21.16 0"/><path d="M8.53 16.11a6 6 0 0 1 6.94 0"/><line x1="12" y1="20" x2="12.01" y2="20"/></svg>`
  };

  function getCurrentPage() {
    const h = window.location.hash ? window.location.hash.slice(1) : '';
    if (['dashboard', 'calib', 'audio', 'wifi'].indexOf(h) >= 0) return h;
    return 'dashboard';
  }

  function renderSidebar() {
    const currentPage = getCurrentPage();

    const sidebarHTML = `
      <aside class="sidebar" id="sidebar">
        <div class="sidebar-header">
          <div class="sidebar-title">DVS CONTROL</div>
        </div>
        <nav class="sidebar-nav">
          ${navItems.map(item => `
            <a href="${item.href}" class="sidebar-link ${item.id === currentPage ? 'active' : ''}" data-page="${item.id}">
              <span class="sidebar-icon">${icons[item.icon]}</span>
              <span>${item.label}</span>
            </a>
          `).join('')}
        </nav>
        <div class="sidebar-footer">
          <button id="sbConnectBtn" class="btn small" style="width:100%;">Conectar</button>
          <div class="sidebar-status">
            <span class="sidebar-dot" id="sidebarDot"></span>
            <span id="sidebarStatus">Desconectado</span>
          </div>
        </div>
      </aside>
      <button class="sidebar-toggle" id="sidebarToggle" aria-label="Toggle menu">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <line x1="3" y1="6" x2="21" y2="6"/>
          <line x1="3" y1="12" x2="21" y2="12"/>
          <line x1="3" y1="18" x2="21" y2="18"/>
        </svg>
      </button>
    `;

    document.body.insertAdjacentHTML('afterbegin', sidebarHTML);

    // Mobile toggle
    const toggle = document.getElementById('sidebarToggle');
    const sidebar = document.getElementById('sidebar');
    toggle.addEventListener('click', () => sidebar.classList.toggle('open'));

    // Close sidebar on link click (mobile)
    document.querySelectorAll('.sidebar-link').forEach(link => {
      link.addEventListener('click', () => {
        if (window.innerWidth <= 900) sidebar.classList.remove('open');
      });
    });

    // Connect button (each page exposes window.connect / window.disconnect)
    const connectBtn = document.getElementById('sbConnectBtn');
    connectBtn.addEventListener('click', () => {
      if (window.__serialConnected) {
        if (typeof window.disconnect === 'function') window.disconnect();
      } else {
        if (typeof window.connect === 'function') window.connect();
      }
    });
  }

  function setActive(page) {
    document.querySelectorAll('.sidebar-link').forEach(l => {
      l.classList.toggle('active', l.dataset.page === page);
    });
  }

  window.Sidebar = {
    init() {
      renderSidebar();
      setActive(getCurrentPage());
      window.addEventListener('hashchange', () => {
        const page = getCurrentPage();
        setActive(page);
        if (typeof window.onSectionChange === 'function') window.onSectionChange(page);
      });
    },
    setConnectionStatus(connected) {
      const dot = document.getElementById('sidebarDot');
      const status = document.getElementById('sidebarStatus');
      const btn = document.getElementById('sbConnectBtn');
      if (dot) dot.classList.toggle('connected', connected);
      if (status) status.textContent = connected ? 'Conectado' : 'Desconectado';
      if (btn) btn.textContent = connected ? 'Desconectar' : 'Conectar';
      window.__serialConnected = connected;
    }
  };

  window.Sidebar.setConnectionStatus(false);

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => Sidebar.init());
  } else {
    Sidebar.init();
  }
})();