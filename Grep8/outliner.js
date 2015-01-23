/**
 * アウトラインを動的に生成する
 * Author: Shintani and Ozono Lab.
 */

;(function(window, document) {
  var kLabelPrefix = 't-section-';
  var kVerticalOffset = 110;

  var TOutliner = function(headline, nav) {
    this.headlineLabels = [];
    this.navElement = document.querySelector(nav);
    this.navElement.classList.add('t-outliner-nav');

    document.body.addEventListener('click', function(event) {
      if (!event.target.classList.contains('t-outliner-link')) {
        return;
      }

      event.preventDefault();
      // アウトラインのリンクがクリックされたら，ラベル部分までスクロール
      // オフセットも考慮する．kVerticalOffset で調整．
      var labelName = event.target.href.split('#').pop();
      var label = this._findLabelByName(labelName);
      if (label !== null) {
        var bounds = label.getBoundingClientRect();
        window.scrollTo(0, bounds.top + document.body.scrollTop - kVerticalOffset);
      }
    }.bind(this));

    var headlineElements = document.querySelectorAll(headline);
    for (var i = 0, ii = headlineElements.length; i < ii; i++) {
      var headEl = headlineElements[i];
      this.register(headEl);
    }
  };

  TOutliner.prototype = {
    _createHeadlineLabel: function() {
      var label = document.createElement('a');
      label.name = kLabelPrefix + (this.headlineLabels.length + 1);
      label.className = 't-outliner-label';
      this.headlineLabels.push(label);
      return label;
    },

    _createLabelLink: function(headline, label) {
      var link = document.createElement('a');
      link.href = '#' + label.name;
      link.textContent = headline.textContent;
      link.className = 't-outliner-link';
      return link;
    },

    _findLabelByName: function(name) {
      for (var i = 0, ii = this.headlineLabels.length; i < ii; i++) {
        if (this.headlineLabels[i].name === name) {
          return this.headlineLabels[i];
        }
      }
      return null;
    },

    register: function(headline) {
      var label = this._createHeadlineLabel();
      headline.parentNode.insertBefore(label, headline);

      var link = this._createLabelLink(headline, label);
      this.navElement.appendChild(link);
    }
  };

  window.TOutliner = TOutliner;

})(window, document);
