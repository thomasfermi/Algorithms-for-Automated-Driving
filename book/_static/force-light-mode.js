// Force light mode and prevent localStorage from switching themes
(function() {
  // Clear any stored theme preference
  localStorage.removeItem("mode");
  localStorage.removeItem("theme");
  
  // Force light mode on document root
  document.documentElement.dataset.mode = "light";
  document.documentElement.dataset.theme = "light";
  document.body.setAttribute("data-default-mode", "light");
  
  // Prevent any future changes via mutation observer
  const observer = new MutationObserver((mutations) => {
    mutations.forEach((mutation) => {
      if (mutation.type === "attributes" && 
          (mutation.attributeName === "data-theme" || 
           mutation.attributeName === "data-mode")) {
        document.documentElement.dataset.mode = "light";
        document.documentElement.dataset.theme = "light";
      }
    });
  });
  
  observer.observe(document.documentElement, {
    attributes: true,
    attributeFilter: ["data-theme", "data-mode"]
  });
})();
